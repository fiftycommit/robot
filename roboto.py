#!/usr/bin/env python3
"""
Robot de football - controle clavier via SSH
- Port 3  : Capteur ultrason fixe a l'avant
- Port A  : Roue gauche
- Port C  : Pelle mecanique
- Port D  : Roue droite

Controles :
  Z / Fleche haut    -> Avancer (maintenir)
  S / Fleche bas     -> Reculer (maintenir)
  Q / Fleche gauche  -> Tourner gauche (maintenir)
  D / Fleche droite  -> Tourner droite (maintenir)
  T                  -> Tir pelle
  H                  -> Bloquer la pelle ou elle est
  1 / 2              -> Regler pelle bas/haut
  3                  -> Monter pelle jusqu'a la position haute de frappe
  0                  -> Mettre position pelle actuelle a zero
  9                  -> Afficher position pelle
  A                  -> Mode auto
  X / rien           -> Stop
  Ctrl+C             -> Quitter
"""
 
import sys
import tty
import termios
import threading
import time
import socket
import json
import math
import mock_strategie as commande
import test_strategie as strategie
from ev3dev2.motor import LargeMotor, Motor, OUTPUT_A, OUTPUT_C, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_3
from ev3dev2.sound import Sound
 
# ── Initialisation ──────────────────────────────────────────────────────────
sound       = Sound()
left_motor  = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_D)
scoop_motor = Motor(OUTPUT_C)        # pelle mecanique (medium ou large)
us_sensor   = UltrasonicSensor(INPUT_3)
us_sensor.mode = 'US-DIST-CM'
 
# ── Parametres ───────────────────────────────────────────────────────────────
DRIVE_SPEED      = 50
TURN_SPEED       = 25
SCOOP_KICK_ANGLE  = 70    # tir relatif depuis la position actuelle, mieux au milieu
SCOOP_UP_SPEED   = 100   # vitesse de frappe vers le haut
SCOOP_DOWN_SPEED = -55   # retour controle vers le bas
SCOOP_ADJUST_STEP = 4    # degres moteur pour le reglage manuel

SHOOT_DIST       = 15.5  # cm - balle collee au robot: on tire
DETECT_DIST      = 38.0  # cm - balle detectee: on approche doucement
OBSTACLE_DIST    = DETECT_DIST
SLOWDOWN_DIST    = 100.0 # cm - a partir d'ici le robot ralentit progressivement
FREE_DIST        = 45.0  # cm - direction consideree libre apres scan
MAX_VALID_DIST   = 200   # cm - au-dela on considere l'echo comme perdu
HIT_CONFIRM      = 1     # tir des la premiere detection
SHOT_COOLDOWN    = 1.5   # secondes - evite de tirer en boucle
POST_SHOT_CHECK_TIME = 0.25 # secondes - laisse le temps a la balle de partir
MIN_AUTO_SPEED   = 18    # vitesse minimale quand un obstacle approche
POST_TURN_FORWARD_TIME = 0.9  # secondes - evite de rescanner en boucle apres virage

STUCK_WATCH_DIST = 130.0 # cm - on surveille le blocage quand la distance devrait changer
STUCK_TIME       = 1.2   # secondes avec roues qui tournent mais distance stable
STUCK_MOTOR_DEG  = 160   # degres moteurs minimum pour dire que les roues ont tourne
STUCK_DIST_DELTA = 4.0   # cm - distance quasi inchangee => robot probablement bloque
LOST_ECHO_TIME   = 0.9   # secondes sans echo avant recuperation
LOST_ECHO_SPEED  = 14    # vitesse prudente si l'echo est instable

SCAN_ANGLE       = 35    # degres robot pour regarder gauche/droite
ESCAPE_ANGLE     = 75    # degres robot si gauche et droite sont bloques
MAX_STUCK_TRIES  = 3     # anti-boucle: stop auto apres plusieurs echecs

# A calibrer: degres moteur necessaires pour tourner le robot de 1 degre.
# Avec des grandes roues EV3 et une voie autour de 14 cm, 2.4 est un bon depart.
TURN_MOTOR_DEG_PER_ROBOT_DEG = 2.4

SPEED_STEP       = 5     # increment pour +/-
SPEED_MIN        = 10
SPEED_MAX        = 100

CAMERA_STATE_PORT = 8081
CAMERA_STATE_MAX_AGE = 0.6
CAMERA_ANGLE_OK = 10
CAMERA_CONTACT_DIST = 23.0
CAMERA_APPROACH_DIST = 80.0
CAMERA_SLOW_SPEED = 22
CAMERA_TURN_SPEED = 22
ULTRASON_OBSTACLE_MAX_DIST = OBSTACLE_DIST
ULTRASON_BALL_MATCH_TOLERANCE = 12.0
VIRTUAL_OBSTACLE_FAR_CM = 9999.0
 
# ── Etat des touches ─────────────────────────────────────────────────────────
keys = {
    'up':    False,
    'down':  False,
    'left':  False,
    'right': False,
}
running      = True
auto_mode    = False
turn_samples = []   # distances enregistrees via 'u' pour calibration
latest_camera_state = None
camera_lock = threading.Lock()
camera_last_shot = 0
camera_escape_dir = 1
 
# ── Moteurs ──────────────────────────────────────────────────────────────────
 
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
    """Tourne le robot sur place. + = droite, - = gauche."""
    motor_degrees = int(abs(angle_deg) * TURN_MOTOR_DEG_PER_ROBOT_DEG)
    motor_degrees = max(30, motor_degrees)

    if angle_deg > 0:
        left_speed = TURN_SPEED
        right_speed = -TURN_SPEED
    else:
        left_speed = -TURN_SPEED
        right_speed = TURN_SPEED

    left_motor.on_for_degrees(
        speed=left_speed,
        degrees=motor_degrees,
        brake=True,
        block=False
    )
    right_motor.on_for_degrees(
        speed=right_speed,
        degrees=motor_degrees,
        brake=True,
        block=True
    )
    left_motor.wait_until_not_moving(timeout=2000)
    stop()
    time.sleep(0.15)
 
def lower_scoop():
    """Ne bouge plus la pelle: on garde la position reglee a la main."""
    scoop_motor.off(brake=True)
    print("\r[PELLE] tenue au milieu position={}       ".format(scoop_motor.position), end='')

def scoop():
    """Tir relatif depuis le milieu: haut a fond puis retour au point de depart."""
    start_pos = scoop_motor.position
    scoop_motor.on_for_degrees(
        speed=SCOOP_UP_SPEED,
        degrees=SCOOP_KICK_ANGLE,
        brake=False,
        block=True
    )
    scoop_motor.on_for_degrees(
        speed=SCOOP_DOWN_SPEED,
        degrees=SCOOP_KICK_ANGLE,
        brake=True,
        block=True
    )
    print("\r[PELLE] tir {} -> {}       ".format(start_pos, scoop_motor.position), end='')

def raise_scoop_to_hit():
    """Monte un peu la pelle pour tester le haut, sans retour automatique."""
    scoop_motor.on_for_degrees(
        speed=SCOOP_UP_SPEED,
        degrees=SCOOP_KICK_ANGLE,
        brake=True,
        block=True
    )
    print("\r[PELLE] position haute test={}       ".format(scoop_motor.position), end='')

def adjust_scoop(degrees):
    """Petit reglage manuel de la pelle pendant les tests."""
    speed = 25 if degrees > 0 else -25
    scoop_motor.on_for_degrees(
        speed=speed,
        degrees=abs(degrees),
        brake=True,
        block=True
    )
    print("\r[PELLE] position={}       ".format(scoop_motor.position), end='')

def reset_scoop_position():
    scoop_motor.reset()
    print("\r[PELLE] position remise a zero       ", end='')

def print_scoop_position():
    print("\r[PELLE] position={}       ".format(scoop_motor.position), end='')

def read_distance(samples=3, delay=0.03):
    """Lit plusieurs fois l'ultrason et renvoie une valeur stable."""
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
    """Vitesse progressive: rapide loin, prudente pres d'un obstacle."""
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

def camera_state_thread():
    global latest_camera_state

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", CAMERA_STATE_PORT))
    sock.settimeout(0.2)

    while running:
        try:
            data, _ = sock.recvfrom(4096)
            state = json.loads(data.decode("utf-8"))
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
    if time.time() - state.get("ts", 0) > CAMERA_STATE_MAX_AGE:
        return None
    if not state.get("robot") or not state.get("ball"):
        return None
    return state

def build_strategy_state(camera_state, distance):
    """Convertit vision + ultrason vers le format de test_strategie."""
    vision_robot = camera_state["robot"]
    vision_ball = camera_state["ball"]

    robot = {
        "x_cm": float(vision_robot["x_cm"]),
        "y_cm": float(vision_robot["y_cm"]),
        "angle": float(vision_robot.get("angle_deg", 0.0)),
    }
    ball = {
        "x_cm": float(vision_ball["x_cm"]),
        "y_cm": float(vision_ball["y_cm"]),
    }

    obstacle = {
        "x_cm": VIRTUAL_OBSTACLE_FAR_CM,
        "y_cm": VIRTUAL_OBSTACLE_FAR_CM,
    }

    if is_valid_distance(distance) and distance <= ULTRASON_OBSTACLE_MAX_DIST:
        dx_ball = ball["x_cm"] - robot["x_cm"]
        dy_ball = ball["y_cm"] - robot["y_cm"]
        ball_dist = math.hypot(dx_ball, dy_ball)
        ball_angle = math.degrees(math.atan2(dy_ball, dx_ball))
        ball_in_front = abs(angle_diff(ball_angle, robot["angle"])) <= strategie.ANGLE_DETECTION
        ultrason_matches_ball = (
            ball_in_front and
            abs(ball_dist - distance) <= ULTRASON_BALL_MATCH_TOLERANCE
        )

        if not ultrason_matches_ball:
            rad = math.radians(robot["angle"])
            obstacle = {
                "x_cm": robot["x_cm"] + math.cos(rad) * distance,
                "y_cm": robot["y_cm"] + math.sin(rad) * distance,
            }

    return robot, ball, obstacle

def apply_strategy_command(cmd, now):
    """Applique une commande mock_strategie sur les vrais moteurs EV3."""
    global camera_last_shot

    action = cmd.get("action", "stop")
    vitesse = int(cmd.get("vitesse", 0))

    if action == "avance":
        drive(vitesse, vitesse)
    elif action == "recule":
        drive(-vitesse, -vitesse)
    elif action == "tourneD":
        drive(vitesse, -vitesse)
    elif action == "tourneG":
        drive(-vitesse, vitesse)
    elif action == "tir":
        stop()
        if now - camera_last_shot >= SHOT_COOLDOWN:
            print("\r\n[STRATEGIE] TIR")
            scoop()
            camera_last_shot = time.time()
        commande.stop()
    else:
        stop()

def camera_auto_step(state, d, now):
    robot, ball, obstacle = build_strategy_state(state, d)
    strategie.jouer_tour(robot, ball, obstacle)
    cmd = commande.get_etat()
    apply_strategy_command(cmd, now)
    return True

# ── Thread lecture clavier ────────────────────────────────────────────────────
 
KEY_TIMEOUT = 0.15  # secondes - si pas de touche recue, on considere relachee
 
def read_keys():
    """
    Lit les touches en continu.
    Met a jour le dictionnaire keys[].
    Une touche est consideree relachee si elle n'est pas re-recue
    dans KEY_TIMEOUT secondes.
    """
    global running, auto_mode, DRIVE_SPEED
 
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
 
    last_time = {}
 
    try:
        import select
        while running:
            r, _, _ = select.select([sys.stdin], [], [], KEY_TIMEOUT)
 
            # Marque les touches comme relachees si timeout depasse
            now = time.time()
            for k in keys:
                if k in last_time and now - last_time[k] > KEY_TIMEOUT:
                    keys[k] = False
 
            if not r:
                continue
 
            ch = sys.stdin.read(1)
 
            # Ctrl+C
            if ch == '\x03':
                running = False
                break
 
            # Sequences fleches
            if ch == '\x1b':
                ch2 = sys.stdin.read(1)
                ch3 = sys.stdin.read(1)
                seq = ch + ch2 + ch3
                if seq == '\x1b[A':
                    keys['up']    = True; last_time['up']    = time.time()
                elif seq == '\x1b[B':
                    keys['down']  = True; last_time['down']  = time.time()
                elif seq == '\x1b[D':
                    keys['left']  = True; last_time['left']  = time.time()
                elif seq == '\x1b[C':
                    keys['right'] = True; last_time['right'] = time.time()
                continue
 
            ch = ch.lower()
 
            if ch == 'z':
                keys['up']    = True; last_time['up']    = time.time()
            elif ch == 's':
                keys['down']  = True; last_time['down']  = time.time()
            elif ch == 'q':
                keys['left']  = True; last_time['left']  = time.time()
            elif ch == 'd':
                keys['right'] = True; last_time['right'] = time.time()
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
                    strategie.reset()
                    commande.stop()
                    print("\r-> Mode AUTO ON       ", end='')
                else:
                    commande.stop()
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
                mn  = min(turn_samples)
                mx  = max(turn_samples)
                print("\r[CALIB] #{} dist={:.1f}cm  (min={:.1f} max={:.1f} moy={:.1f})        ".format(
                    len(turn_samples), d, mn, mx, avg), end='')
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
 
# ── Thread mode auto ──────────────────────────────────────────────────────────

def scan_direction():
    """
    Le capteur est fixe: pour comparer gauche/droite, on tourne le robot.
    Retourne l'angle a prendre (-SCAN_ANGLE ou +SCAN_ANGLE), ou None si bloque.
    """
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
    """Recule, compare gauche/droite, puis s'oriente vers le meilleur cote."""
    backward(0.7 + 0.2 * min(stuck_count, 3))
    best_angle = scan_direction()

    if best_angle is not None:
        turn_by_angle(best_angle)
        return escape_dir, True

    turn_by_angle(ESCAPE_ANGLE * escape_dir)
    return -escape_dir, False

def auto_thread():
    global auto_mode
    last_log       = 0
    close_count    = 0
    obstacle_count = 0
    stuck_count    = 0
    escape_dir     = 1
    last_shot      = 0
    last_valid_d   = SLOWDOWN_DIST
    forward_until  = 0
    stuck_start    = 0
    stuck_start_d  = 0
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
                obstacle_count = 0
                stuck_start = 0
                lost_echo_start = 0
                forward_until = time.time() + POST_TURN_FORWARD_TIME
            elif d <= SHOOT_DIST:
                close_count += 1
                obstacle_count = 0
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
                obstacle_count = 0
                speed = LOST_ECHO_SPEED if lost_echo else max(MIN_AUTO_SPEED, auto_speed_for_distance(d))
                drive(speed, speed)
            elif d <= DETECT_DIST:
                obstacle_count = 0
                close_count = 0
                drive(MIN_AUTO_SPEED, MIN_AUTO_SPEED)
            else:
                close_count = 0
                obstacle_count = 0
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
 
# ── Thread controle moteurs ───────────────────────────────────────────────────
 
def motor_thread():
    while running:
        if auto_mode:
            time.sleep(0.05)
            continue
 
        up    = keys['up']
        down  = keys['down']
        left  = keys['left']
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
 
# ── Main ──────────────────────────────────────────────────────────────────────
 
def main():
    global running
 
    sound.speak("Pret")
 
    print("=== ROBOT FOOTBALL ===")
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
    print("U         : Enregistrer distance (calibration)")
    print("Ctrl+C    : Quitter")
    print("======================")
 
    t_auto   = threading.Thread(target=auto_thread)
    t_motor  = threading.Thread(target=motor_thread)
    t_camera = threading.Thread(target=camera_state_thread)
    t_auto.daemon   = True
    t_motor.daemon  = True
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
            print("=== CALIBRATION ===")
            print("Echantillons ({}): {}".format(
                len(turn_samples),
                ", ".join("{:.1f}".format(s) for s in turn_samples)))
            print("Min = {:.1f}cm  Max = {:.1f}cm  Moyenne = {:.1f}cm".format(
                min(turn_samples), max(turn_samples), avg))
 
if __name__ == '__main__':
    main()
