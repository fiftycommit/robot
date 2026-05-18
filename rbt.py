#!/usr/bin/env python3
"""
Robot de football - controle clavier via SSH
- Port 1  : Capteur ultrason
- Port A  : Roue gauche
- Port B  : Pelle mecanique
- Port D  : Roue droite

Controles :
  Z / Fleche haut    -> Avancer (maintenir)
  S / Fleche bas     -> Reculer (maintenir)
  Q / Fleche gauche  -> Tourner gauche (maintenir)
  D / Fleche droite  -> Tourner droite (maintenir)
  T                  -> Tir pelle
  A                  -> Mode auto
  X / rien           -> Stop
  Ctrl+C             -> Quitter
"""
 
import sys
import tty
import termios
import threading
import time
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_1
from ev3dev2.sound import Sound
 
# ── Initialisation ──────────────────────────────────────────────────────────
sound       = Sound()
left_motor  = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_D)
scoop_motor = LargeMotor(OUTPUT_B)   # pelle mecanique
us_sensor   = UltrasonicSensor(INPUT_1)
us_sensor.mode = 'US-DIST-CM'
 
# ── Parametres ───────────────────────────────────────────────────────────────
DRIVE_SPEED   = 50
TURN_SPEED    = 25
SCOOP_ANGLE   = 180   # degres de tir de la pelle
BALL_DIST     = 15.5  # cm - distance frappe auto
SPEED_STEP    = 5     # increment pour +/-
SPEED_MIN     = 10
SPEED_MAX     = 100
 
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
 
# ── Moteurs ──────────────────────────────────────────────────────────────────
 
def drive(left, right):
    left_motor.on(left)
    right_motor.on(right)
 
def stop():
    left_motor.off(brake=True)
    right_motor.off(brake=True)
 
def scoop():
    """Tir de la pelle: rotation a fond puis relache (retombe naturellement)."""
    scoop_motor.on_for_degrees(speed=100, degrees=SCOOP_ANGLE, brake=False, block=True)

def dist():
    d = us_sensor.distance_centimeters
    return d if d is not None else 999
 
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
                mn  = min(turn_samples)
                mx  = max(turn_samples)
                print("\r[CALIB] #{} dist={:.1f}cm  (min={:.1f} max={:.1f} moy={:.1f})        ".format(
                    len(turn_samples), d, mn, mx, avg), end='')
 
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
 
# ── Thread mode auto ──────────────────────────────────────────────────────────
 
MAX_VALID_DIST  = 200   # cm - au-dela on considere l'echo comme perdu
HIT_CONFIRM     = 2     # nb de lectures basses consecutives pour declencher stop

def auto_thread():
    global auto_mode
    last_log  = 0
    last_d    = None   # derniere distance valide connue
    hit_count = 0      # compteur de lectures basses consecutives
    while running:
        if auto_mode:
            raw = us_sensor.distance_centimeters

            # Filtre: ignore None et valeurs hors portee (echo perdu)
            if raw is None or raw >= MAX_VALID_DIST:
                valid = False
                d = last_d if last_d is not None else 999
            else:
                valid = True
                d = raw
                last_d = raw

            now = time.time()
            if now - last_log >= 0.2:
                tag = "AVANCE" if d > BALL_DIST else "PROCHE({}/{})".format(hit_count + 1, HIT_CONFIRM)
                flag = "" if valid else " [echo perdu, utilise last={}]".format(last_d)
                print("\r\n[AUTO] raw={} d={:.2f}cm seuil={:.2f}cm -> {}{}".format(
                    raw, d, BALL_DIST, tag, flag), end='')
                last_log = now

            if valid and d <= BALL_DIST:
                hit_count += 1
                if hit_count >= HIT_CONFIRM:
                    stop()
                    print("\r\n[AUTO] >>> Obstacle confirme a {:.2f}cm (seuil {:.2f}) - ARRET".format(d, BALL_DIST))
                    auto_mode = False
                    hit_count = 0
            else:
                hit_count = 0
                drive(DRIVE_SPEED, DRIVE_SPEED)
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
    print("A         : Mode auto")
    print("X         : Stop")
    print("P / M     : Ajuster DRIVE_SPEED (+/- {})".format(SPEED_STEP))
    print("U         : Enregistrer distance (calibration)")
    print("Ctrl+C    : Quitter")
    print("======================")
 
    t_auto  = threading.Thread(target=auto_thread)
    t_motor = threading.Thread(target=motor_thread)
    t_auto.daemon  = True
    t_motor.daemon = True
    t_auto.start()
    t_motor.start()
 
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