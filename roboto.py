#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot de football - controle clavier via SSH
"""

import sys
import tty
import termios
import threading
import time
import socket
import json
import math

from ev3dev2.motor import LargeMotor, Motor, OUTPUT_A, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_3
from ev3dev2.sound import Sound


sound = Sound()
left_motor = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_D)
scoop_motor = Motor(OUTPUT_C)
us_sensor = UltrasonicSensor(INPUT_3)
us_sensor.mode = 'US-DIST-CM'


DRIVE_SPEED = 50
TURN_SPEED = 25

SCOOP_KICK_ANGLE = 70
SCOOP_UP_SPEED = 100
SCOOP_DOWN_SPEED = -55
SCOOP_ADJUST_STEP = 4

SHOOT_DIST = 15.5
DETECT_DIST = 38.0
OBSTACLE_DIST = DETECT_DIST
SLOWDOWN_DIST = 100.0
FREE_DIST = 45.0
MAX_VALID_DIST = 200
HIT_CONFIRM = 1
SHOT_COOLDOWN = 1.5
POST_SHOT_CHECK_TIME = 0.25
MIN_AUTO_SPEED = 18
POST_TURN_FORWARD_TIME = 0.9

STUCK_WATCH_DIST = 130.0
STUCK_TIME = 1.2
STUCK_MOTOR_DEG = 160
STUCK_DIST_DELTA = 4.0
LOST_ECHO_TIME = 0.9
LOST_ECHO_SPEED = 14

SCAN_ANGLE = 35
ESCAPE_ANGLE = 75
MAX_STUCK_TRIES = 3
TURN_MOTOR_DEG_PER_ROBOT_DEG = 2.4

SPEED_STEP = 5
SPEED_MIN = 10
SPEED_MAX = 100

CAMERA_STATE_PORT = 8081
CAMERA_STATE_MAX_AGE = 0.6
CAMERA_ANGLE_OK = 10
CAMERA_CONTACT_DIST = 23.0
CAMERA_APPROACH_DIST = 80.0
CAMERA_SLOW_SPEED = 22
CAMERA_TURN_SPEED = 22
CAMERA_VERBOSE_INTERVAL = 0.25

AUTO_VERBOSE = True

ARENA_W_CM = 301.0
ARENA_H_CM = 390.0
ARENA_MARGIN_CM = 35.0
ARENA_CRITICAL_MARGIN_CM = 18.0
ARENA_CENTER_X = ARENA_W_CM / 2.0
ARENA_CENTER_Y = ARENA_H_CM / 2.0
ARENA_ESCAPE_SPEED = 20


keys = {
    'up': False,
    'down': False,
    'left': False,
    'right': False,
}

running = True
auto_mode = False
turn_samples = []
latest_camera_state = None
camera_lock = threading.Lock()
camera_last_shot = 0
camera_escape_dir = 1
last_camera_verbose = 0


def drive(left, right):
    left_motor.on(left)
    right_motor.on(right)


def stop():
    left_motor.off(brake=True)
    right_motor.off(brake=True)


def backward(duration=0.35):
    drive(-DRIVE_SPEED, -DRIVE_SPEED)
    time.sleep(duration)
    stop()
    time.sleep(0.1)


def turn_by_angle(angle_deg):
    motor_degrees = int(abs(angle_deg) * TURN_MOTOR_DEG_PER_ROBOT_DEG)
    motor_degrees = max(30, motor_degrees)

    if angle_deg > 0:
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED
    else:
        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED

    left_motor.on_for_degrees(speed=left_speed, degrees=motor_degrees, brake=True, block=False)
    right_motor.on_for_degrees(speed=right_speed, degrees=motor_degrees, brake=True, block=True)
    left_motor.wait_until_not_moving(timeout=2000)
    stop()
    time.sleep(0.15)


def lower_scoop():
    scoop_motor.off(brake=True)
    print("\r[PELLE] tenue au milieu position={}       ".format(scoop_motor.position), end='')


def scoop():
    start_pos = scoop_motor.position
    scoop_motor.on_for_degrees(speed=SCOOP_UP_SPEED, degrees=SCOOP_KICK_ANGLE, brake=False, block=True)
    scoop_motor.on_for_degrees(speed=SCOOP_DOWN_SPEED, degrees=SCOOP_KICK_ANGLE, brake=True, block=True)
    print("\r[PELLE] tir {} -> {}       ".format(start_pos, scoop_motor.position), end='')


def raise_scoop_to_hit():
    scoop_motor.on_for_degrees(speed=SCOOP_UP_SPEED, degrees=SCOOP_KICK_ANGLE, brake=True, block=True)
    print("\r[PELLE] position haute test={}       ".format(scoop_motor.position), end='')


def adjust_scoop(degrees):
    speed = 25 if degrees > 0 else -25
    scoop_motor.on_for_degrees(speed=speed, degrees=abs(degrees), brake=True, block=True)
    print("\r[PELLE] position={}       ".format(scoop_motor.position), end='')


def reset_scoop_position():
    scoop_motor.reset()
    print("\r[PELLE] position remise a zero       ", end='')


def print_scoop_position():
    print("\r[PELLE] position={}       ".format(scoop_motor.position), end='')


def read_distance(samples=3, delay=0.03):
    values = []
    for _ in range(samples):
        d = us_sensor.distance_centimeters
        if d is not None and d < MAX_VALID_DIST:
            values.append(d)
        time.sleep(delay)

    if not values:
        return 999

    values.sort()
    return values[len(values) // 2]


def dist():
    return read_distance(samples=1, delay=0)


def auto_speed_for_distance(distance):
    if distance >= SLOWDOWN_DIST:
        return DRIVE_SPEED
    if distance <= OBSTACLE_DIST:
        return MIN_AUTO_SPEED

    ratio = (distance - OBSTACLE_DIST) / float(SLOWDOWN_DIST - OBSTACLE_DIST)
    return int(MIN_AUTO_SPEED + ratio * (DRIVE_SPEED - MIN_AUTO_SPEED))


def is_valid_distance(distance):
    return distance is not None and distance < MAX_VALID_DIST


def angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def arena_edges(robot):
    x = robot["x_cm"]
    y = robot["y_cm"]
    return {
        "G": x,
        "D": ARENA_W_CM - x,
        "H": y,
        "B": ARENA_H_CM - y,
    }


def log_camera_verbose(robot, ball, ball_dist, diff):
    global last_camera_verbose

    if not AUTO_VERBOSE:
        return

    now = time.time()
    if now - last_camera_verbose < CAMERA_VERBOSE_INTERVAL:
        return

    edges = arena_edges(robot)
    print(
        "\r\n[CAMERA] robot=({:.1f},{:.1f}) angle={:.1f} "
        "balle=({:.1f},{:.1f}) dist={:.1f} diff={:.1f} "
        "bords G={:.1f} D={:.1f} H={:.1f} B={:.1f}".format(
            robot["x_cm"],
            robot["y_cm"],
            robot.get("angle_deg", 0.0),
            ball["x_cm"],
            ball["y_cm"],
            ball_dist,
            diff,
            edges["G"],
            edges["D"],
            edges["H"],
            edges["B"],
        ),
        end=''
    )

    last_camera_verbose = now


def keep_inside_arena(robot):
    edges = arena_edges(robot)
    closest = min(edges.values())

    if closest > ARENA_MARGIN_CM:
        return False

    target_angle = math.degrees(math.atan2(
        ARENA_CENTER_Y - robot["y_cm"],
        ARENA_CENTER_X - robot["x_cm"]
    ))
    robot_angle = robot.get("angle_deg", 0.0)
    diff = angle_diff(target_angle, robot_angle)

    level = "CRITIQUE" if closest <= ARENA_CRITICAL_MARGIN_CM else "MARGE"
    print(
        "\r\n[LIMITE] {} robot=({:.1f},{:.1f}) angle={:.1f} "
        "bords G={:.1f} D={:.1f} H={:.1f} B={:.1f} -> centre diff={:.1f}".format(
            level,
            robot["x_cm"],
            robot["y_cm"],
            robot_angle,
            edges["G"],
            edges["D"],
            edges["H"],
            edges["B"],
            diff,
        ),
        end=''
    )

    if abs(diff) > CAMERA_ANGLE_OK:
        if diff > 0:
            drive(CAMERA_TURN_SPEED, -CAMERA_TURN_SPEED)
        else:
            drive(-CAMERA_TURN_SPEED, CAMERA_TURN_SPEED)
    else:
        drive(ARENA_ESCAPE_SPEED, ARENA_ESCAPE_SPEED)

    return True


def camera_state_thread():
    global latest_camera_state

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", CAMERA_STATE_PORT))
    sock.settimeout(0.2)

    print("\r\n[CAMERA] attente UDP port {}".format(CAMERA_STATE_PORT))

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            state = json.loads(data.decode("utf-8"))
            state["_rx_ts"] = time.time()
            with camera_lock:
                latest_camera_state = state
        except socket.timeout:
            continue
        except Exception as exc:
            print("\r\n[CAMERA] paquet ignore: {}".format(exc))

    sock.close()


def get_fresh_camera_state():
    with camera_lock:
        state = latest_camera_state

    if not state:
        return None
    if time.time() - state.get("_rx_ts", 0) > CAMERA_STATE_MAX_AGE:
        return None
    if not state.get("robot") or not state.get("ball"):
        return None
    return state


def camera_auto_step(state, d, now):
    global camera_last_shot, camera_escape_dir

    robot = state["robot"]
    ball = state["ball"]

    dx = ball["x_cm"] - robot["x_cm"]
    dy = ball["y_cm"] - robot["y_cm"]
    ball_dist = math.hypot(dx, dy)
    target_angle = math.degrees(math.atan2(dy, dx))
    diff = angle_diff(target_angle, robot.get("angle_deg", 0.0))

    log_camera_verbose(robot, ball, ball_dist, diff)

    if keep_inside_arena(robot):
        return True

    if d <= OBSTACLE_DIST and ball_dist > CAMERA_APPROACH_DIST:
        stop()
        print("\r\n[CAMERA] obstacle ultrason devant, balle ailleurs - securite")
        return False

    if ball_dist <= CAMERA_CONTACT_DIST and now - camera_last_shot >= SHOT_COOLDOWN:
        stop()
        print("\r\n[CAMERA] balle au contact ({:.1f}cm) - TIR".format(ball_dist))
        scoop()
        time.sleep(POST_SHOT_CHECK_TIME)
        after_shot_d = read_distance()
        camera_last_shot = time.time()

        if after_shot_d <= DETECT_DIST:
            print("\r\n[CAMERA] objet encore devant apres tir ({:.1f}cm) - RECUL + SCAN".format(after_shot_d))
            camera_escape_dir, _ = recover_from_obstacle(1, camera_escape_dir)

        return True

    if abs(diff) > CAMERA_ANGLE_OK:
        if diff > 0:
            drive(CAMERA_TURN_SPEED, -CAMERA_TURN_SPEED)
        else:
            drive(-CAMERA_TURN_SPEED, CAMERA_TURN_SPEED)
    else:
        speed = CAMERA_SLOW_SPEED if ball_dist <= DETECT_DIST else DRIVE_SPEED
        drive(speed, speed)

    return True


KEY_TIMEOUT = 0.15


def read_keys():
    global running, auto_mode, DRIVE_SPEED

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)

    last_time = {}

    try:
        import select

        while running:
            r, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)

            now = time.time()
            for k in keys:
                if k in last_time and now - last_time[k] > KEY_TIMEOUT:
                    keys[k] = False

            if not r:
                continue

            ch = sys.stdin.read(1)

            if ch == '\x03':
                running = False
                break

            if ch == '\x1b':
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                seq = ch + ch2 + ch3

                if seq == '\x1b[A':
                    keys['up'] = True
                    last_time['up'] = time.time()
                elif seq == '\x1b[B':
                    keys['down'] = True
                    last_time['down'] = time.time()
                elif seq == '\x1b[D':
                    keys['left'] = True
                    last_time['left'] = time.time()
                elif seq == '\x1b[C':
                    keys['right'] = True
                    last_time['right'] = time.time()
                continue

            ch = ch.lower()

            if ch == 'z':
                keys['up'] = True
                last_time['up'] = time.time()
            elif ch == 's':
                keys['down'] = True
                last_time['down'] = time.time()
            elif ch == 'q':
                keys['left'] = True
                last_time['left'] = time.time()
            elif ch == 'd':
                keys['right'] = True
                last_time['right'] = time.time()
            elif ch == 't':
                t = threading.Thread(target=scoop)
                t.daemon = True
                t.start()
            elif ch == 'h':
                t = threading.Thread(target=lower_scoop)
                t.daemon = True
                t.start()
            elif ch == '1':
                t = threading.Thread(target=adjust_scoop, args=(-SCOOP_ADJUST_STEP,))
                t.daemon = True
                t.start()
            elif ch == '2':
                t = threading.Thread(target=adjust_scoop, args=(SCOOP_ADJUST_STEP,))
                t.daemon = True
                t.start()
            elif ch == '3':
                t = threading.Thread(target=raise_scoop_to_hit)
                t.daemon = True
                t.start()
            elif ch == '0':
                reset_scoop_position()
            elif ch == '9':
                print_scoop_position()
            elif ch == 'a':
                auto_mode = not auto_mode
                if auto_mode:
                    print("\r-> Mode AUTO ON       ", end='')
                else:
                    print("\r-> Mode AUTO OFF      ", end='')
            elif ch == 'x':
                for k in keys:
                    keys[k] = False
                auto_mode = False
            elif ch == 'p':
                DRIVE_SPEED = min(SPEED_MAX, DRIVE_SPEED + SPEED_STEP)
                print("\r-> DRIVE_SPEED = {}       ".format(DRIVE_SPEED), end='')
            elif ch == 'm':
                DRIVE_SPEED = max(SPEED_MIN, DRIVE_SPEED - SPEED_STEP)
                print("\r-> DRIVE_SPEED = {}       ".format(DRIVE_SPEED), end='')
            elif ch == 'u':
                d = dist()
                turn_samples.append(d)
                avg = sum(turn_samples) / len(turn_samples)
                mn = min(turn_samples)
                mx = max(turn_samples)
                print("\r[CALIB] #{} dist={:.1f}cm (min={:.1f} max={:.1f} moy={:.1f})        ".format(
                    len(turn_samples), d, mn, mx, avg), end='')

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def scan_direction():
    stop()
    time.sleep(0.1)

    turn_by_angle(-SCAN_ANGLE)
    left_d = read_distance(samples=4)

    turn_by_angle(SCAN_ANGLE * 2)
    right_d = read_distance(samples=4)

    turn_by_angle(-SCAN_ANGLE)

    left_valid = is_valid_distance(left_d)
    right_valid = is_valid_distance(right_d)

    if left_valid and right_valid:
        choose_left = left_d >= right_d
        best_angle = -SCAN_ANGLE if choose_left else SCAN_ANGLE
        best_d = left_d if choose_left else right_d
    elif left_valid:
        best_angle = -SCAN_ANGLE
        best_d = left_d
    elif right_valid:
        best_angle = SCAN_ANGLE
        best_d = right_d
    else:
        best_angle = None
        best_d = 0

    print("\r\n[AUTO] scan gauche={:.1f}cm droite={:.1f}cm -> {}".format(
        left_d,
        right_d,
        "inconnu" if best_angle is None else ("bloque" if best_d < FREE_DIST else best_angle)
    ))

    if best_angle is None or best_d < FREE_DIST:
        return None

    return best_angle


def recover_from_obstacle(stuck_count, escape_dir):
    backward(0.7 + 0.2 * min(stuck_count, 3))
    best_angle = scan_direction()

    if best_angle is not None:
        turn_by_angle(best_angle)
        return escape_dir, True

    turn_by_angle(ESCAPE_ANGLE * escape_dir)
    return -escape_dir, False


def auto_thread():
    global auto_mode

    last_log = 0
    close_count = 0
    stuck_count = 0
    escape_dir = 1
    last_shot = 0
    last_valid_d = SLOWDOWN_DIST
    forward_until = 0
    stuck_start = 0
    stuck_start_d = 0
    stuck_left_pos = 0
    stuck_right_pos = 0
    lost_echo_start = 0

    while running:
        if auto_mode:
            raw_d = read_distance()
            now = time.time()

            lost_echo = raw_d >= 999
            if lost_echo:
                if lost_echo_start == 0:
                    lost_echo_start = now
                d = last_valid_d
            else:
                lost_echo_start = 0
                d = raw_d
                last_valid_d = raw_d

            camera_state = get_fresh_camera_state()
            if camera_state and camera_auto_step(camera_state, d, now):
                time.sleep(0.05)
                continue

            if now - last_log >= 0.2:
                if d <= SHOOT_DIST:
                    tag = "TIR"
                elif now < forward_until:
                    tag = "AVANCE_POST_VIRAGE"
                elif d <= DETECT_DIST:
                    tag = "APPROCHE_LENTE"
                else:
                    tag = "AVANCE"

                speed = LOST_ECHO_SPEED if lost_echo else auto_speed_for_distance(d)
                echo = " echo_perdu" if lost_echo else ""

                print("\r\n[AUTO] d={:.1f}cm v={} tir<={:.1f} obstacle<={:.1f} -> {}{}".format(
                    d, speed, SHOOT_DIST, DETECT_DIST, tag, echo), end='')
                last_log = now

            if lost_echo and lost_echo_start and now - lost_echo_start >= LOST_ECHO_TIME:
                stop()
                stuck_count += 1
                print("\r\n[AUTO] >>> Echo perdu trop longtemps - RECUL + ANALYSE")
                backward(0.7 + 0.2 * min(stuck_count, 3))
                best_angle = scan_direction()

                if best_angle is not None:
                    turn_by_angle(best_angle)
                else:
                    turn_by_angle(ESCAPE_ANGLE * escape_dir)
                    escape_dir *= -1

                close_count = 0
                stuck_start = 0
                lost_echo_start = 0
                forward_until = time.time() + POST_TURN_FORWARD_TIME

            elif d <= SHOOT_DIST:
                close_count += 1

                if close_count >= HIT_CONFIRM and now - last_shot >= SHOT_COOLDOWN:
                    stop()
                    print("\r\n[AUTO] >>> Balle au contact a {:.1f}cm - TIR".format(d))
                    scoop()
                    time.sleep(POST_SHOT_CHECK_TIME)
                    after_shot_d = read_distance()
                    last_shot = time.time()
                    close_count = 0
                    forward_until = 0

                    if after_shot_d <= DETECT_DIST:
                        stuck_count += 1
                        print("\r\n[AUTO] >>> Objet encore devant apres tir ({:.1f}cm) - RECUL + SCAN".format(
                            after_shot_d))
                        escape_dir, found_free = recover_from_obstacle(stuck_count, escape_dir)

                        if found_free:
                            stuck_count = 0
                        elif stuck_count >= MAX_STUCK_TRIES:
                            stop()
                            sound.speak("Bloque")
                            print("\r\n[AUTO] >>> Bloque apres {} essais - AUTO OFF".format(stuck_count))
                            auto_mode = False

                        forward_until = time.time() + POST_TURN_FORWARD_TIME
                    else:
                        stuck_count = 0
                        backward(0.2)
                else:
                    stop()

                stuck_start = 0

            elif now < forward_until:
                close_count = 0
                speed = LOST_ECHO_SPEED if lost_echo else max(MIN_AUTO_SPEED, auto_speed_for_distance(d))
                drive(speed, speed)

            elif d <= DETECT_DIST:
                close_count = 0
                drive(MIN_AUTO_SPEED, MIN_AUTO_SPEED)

            else:
                close_count = 0
                speed = LOST_ECHO_SPEED if lost_echo else auto_speed_for_distance(d)
                drive(speed, speed)

            moving_forward = auto_mode and d > SHOOT_DIST and (
                now < forward_until or d <= DETECT_DIST or d > OBSTACLE_DIST
            )

            if moving_forward and d <= STUCK_WATCH_DIST and not lost_echo:
                if stuck_start == 0:
                    stuck_start = now
                    stuck_start_d = d
                    stuck_left_pos = left_motor.position
                    stuck_right_pos = right_motor.position
                else:
                    motor_delta = (
                        abs(left_motor.position - stuck_left_pos) +
                        abs(right_motor.position - stuck_right_pos)
                    )
                    dist_delta = abs(d - stuck_start_d)

                    if dist_delta > STUCK_DIST_DELTA:
                        stuck_start = now
                        stuck_start_d = d
                        stuck_left_pos = left_motor.position
                        stuck_right_pos = right_motor.position
                    elif now - stuck_start >= STUCK_TIME and motor_delta >= STUCK_MOTOR_DEG:
                        stop()
                        stuck_count += 1
                        print("\r\n[AUTO] >>> Bloque: roues={}deg distance_delta={:.1f}cm - RECUL + ANALYSE".format(
                            motor_delta, dist_delta))

                        escape_dir, found_free = recover_from_obstacle(stuck_count, escape_dir)
                        if found_free:
                            stuck_count = 0

                        forward_until = time.time() + POST_TURN_FORWARD_TIME
                        stuck_start = 0
            else:
                stuck_start = 0

        time.sleep(0.05)


def motor_thread():
    while running:
        if auto_mode:
            time.sleep(0.05)
            continue

        up = keys['up']
        down = keys['down']
        left = keys['left']
        right = keys['right']

        if up and left:
            drive(DRIVE_SPEED // 2, DRIVE_SPEED)
        elif up and right:
            drive(DRIVE_SPEED, DRIVE_SPEED // 2)
        elif down and left:
            drive(-DRIVE_SPEED // 2, -DRIVE_SPEED)
        elif down and right:
            drive(-DRIVE_SPEED, -DRIVE_SPEED // 2)
        elif up:
            drive(DRIVE_SPEED, DRIVE_SPEED)
        elif down:
            drive(-DRIVE_SPEED, -DRIVE_SPEED)
        elif left:
            drive(-TURN_SPEED, TURN_SPEED)
        elif right:
            drive(TURN_SPEED, -TURN_SPEED)
        else:
            stop()

        time.sleep(0.05)


def main():
    global running

    sound.speak("Pret")

    print("ROBOT FOOTBALL")
    print("Z/Haut    : Avancer")
    print("S/Bas     : Reculer")
    print("Q/Gauche  : Gauche")
    print("D/Droite  : Droite")
    print("T         : Tir pelle")
    print("H         : Pelle en bas")
    print("1 / 2     : Regler pelle bas / haut")
    print("3         : Monter pelle en position haute")
    print("0 / 9     : Zero pelle / afficher position")
    print("A         : Mode auto")
    print("X         : Stop")
    print("P / M     : Ajuster DRIVE_SPEED (+/- {})".format(SPEED_STEP))
    print("U         : Enregistrer distance calibration")
    print("Ctrl+C    : Quitter")
    print("                           ")

    t_auto = threading.Thread(target=auto_thread)
    t_motor = threading.Thread(target=motor_thread)
    t_camera = threading.Thread(target=camera_state_thread)

    t_auto.daemon = True
    t_motor.daemon = True
    t_camera.daemon = True

    t_auto.start()
    t_motor.start()
    t_camera.start()

    try:
        read_keys()
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        stop()
        scoop_motor.off(brake=True)
        sound.speak("Arret")
        print("\nRobot arrete.")

        if turn_samples:
            avg = sum(turn_samples) / len(turn_samples)
            print("Echantillons ({}): {}".format(
                len(turn_samples),
                ", ".join("{:.1f}".format(s) for s in turn_samples)))
            print("Min = {:.1f}cm  Max = {:.1f}cm  Moyenne = {:.1f}cm".format(
                min(turn_samples), max(turn_samples), avg))


if __name__ == '__main__':
    main()