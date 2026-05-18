#!/usr/bin/env python3
"""
Robot avec capteur ultrason rotatif - detection confirmee
- Port 1  : Capteur ultrason
- Port A  : Roue gauche
- Port D  : Roue droite
- Port B  : Moteur de rotation du capteur
 
Limites mecaniques calibrees : +108 / -109 degres moteur
"""
 
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_1
from ev3dev2.sound import Sound
import time
 
# ── Initialisation ──────────────────────────────────────────────────────────
sound       = Sound()
left_motor  = LargeMotor(OUTPUT_A)
right_motor = LargeMotor(OUTPUT_D)
scan_motor  = MediumMotor(OUTPUT_B)
us_sensor   = UltrasonicSensor(INPUT_1)
us_sensor.mode = 'US-DIST-CM'
 
# ── Parametres ───────────────────────────────────────────────────────────────
DRIVE_SPEED      = 50
TURN_SPEED       = 30
SCAN_SPEED       = 40
 
OBSTACLE_DIST    = 25    # cm - seuil obstacle
FREE_DIST        = 45    # cm - zone vraiment libre
FRONT_DEAD       = 40    # degres ignores devant lors de la recherche
 
CONFIRM_COUNT    = 3     # lectures consecutives pour confirmer un obstacle
 
SCAN_MAX         = 98    # degres max calibres (+108/-109 avec marge de 10)
SCAN_STEP        = 20    # pas du balayage
TURN_DURATION    = 0.7   # secondes pour 90 degres
 
# ── Moteurs ──────────────────────────────────────────────────────────────────
 
def drive(left, right):
    left_motor.on(left)
    right_motor.on(right)
 
def stop():
    left_motor.off(brake=True)
    right_motor.off(brake=True)
 
def backward(duration=0.5):
    drive(-DRIVE_SPEED, -DRIVE_SPEED)
    time.sleep(duration)
    stop()
    time.sleep(0.1)
 
def turn_by_angle(angle_deg):
    """Tourne les roues proportionnellement a l angle. + = droite, - = gauche."""
    t = abs(angle_deg) / 90.0 * TURN_DURATION
    t = max(0.3, min(t, 1.3))
    if angle_deg > 0:
        drive(TURN_SPEED, -TURN_SPEED)
    else:
        drive(-TURN_SPEED, TURN_SPEED)
    time.sleep(t)
    stop()
    time.sleep(0.1)
 
def sensor_goto(angle, speed=SCAN_SPEED):
    # Limite l angle aux bornes mecaniques avec marge
    angle = max(-SCAN_MAX, min(SCAN_MAX, angle))
    scan_motor.run_to_abs_pos(position_sp=angle, speed_sp=speed, stop_action='hold')
    scan_motor.wait_until_not_moving(timeout=2000)
 
def dist():
    d = us_sensor.distance_centimeters
    return d if d is not None else 999
 
# ── Detection confirmee ───────────────────────────────────────────────────────
 
class ObstacleDetector:
    def __init__(self, threshold, count):
        self.threshold = threshold
        self.count     = count
        self._hits     = 0
 
    def update(self, distance):
        if distance < self.threshold:
            self._hits += 1
        else:
            self._hits = 0
        return self._hits >= self.count
 
    def reset(self):
        self._hits = 0
 
# ── Recherche direction libre ─────────────────────────────────────────────────
 
def find_free():
    """
    Cherche la premiere direction libre en alternant droite/gauche.
    Ignore la zone frontale. Scan rapide.
    """
    rights = list(range(FRONT_DEAD, SCAN_MAX + 1, 30))
    lefts  = list(range(-FRONT_DEAD, -SCAN_MAX - 1, -30))
 
    pairs = []
    for r, l in zip(rights, lefts):
        pairs.append(r)
        pairs.append(l)
    for r in rights[len(lefts):]:
        pairs.append(r)
    for l in lefts[len(rights):]:
        pairs.append(l)
 
    for angle in pairs:
        sensor_goto(angle, speed=70)
        d = dist()
        print("  Scan {} deg : {:.0f}cm".format(angle, d))
        if d >= FREE_DIST:
            return angle
 
    return None
 
def escape():
    """Deblocage progressif : recule de plus en plus et retente."""
    for recul in [0.5, 1.0, 1.5]:
        backward(recul)
        free = find_free()
        if free is not None:
            return free
    return None
 
# ── Boucle principale ─────────────────────────────────────────────────────────
 
def main():
    scan_motor.reset()
    sensor_goto(0)
    sound.speak("Pret")
    print("Pret")
 
    detector    = ObstacleDetector(threshold=OBSTACLE_DIST, count=CONFIRM_COUNT)
    sweep_dir   = 1
    sweep_angle = 0
 
    try:
        while True:
            d         = dist()
            confirmed = detector.update(d)
 
            print("Angle: {}  Dist: {:.0f}cm  Hits: {}".format(
                scan_motor.position, d, detector._hits))
 
            if confirmed:
                # ── Obstacle confirme ──
                stop()
                sound.beep()
                detector.reset()
 
                free_angle = find_free()
 
                if free_angle is None:
                    sound.speak("Bloque")
                    free_angle = escape()
 
                if free_angle is not None:
                    print("Libre a {} deg".format(free_angle))
                    turn_by_angle(free_angle)
                    sensor_goto(0)
                    sweep_angle = 0
                    sweep_dir   = 1
                    sound.speak("Go")
                else:
                    # Demi-tour en dernier recours
                    print("Demi-tour")
                    sound.speak("Demi tour")
                    turn_by_angle(180)
                    sensor_goto(0)
                    sweep_angle = 0
                    sweep_dir   = 1
 
            else:
                # ── Avance en balayant ──
                if d >= FREE_DIST:
                    speed = DRIVE_SPEED
                else:
                    ratio = (d - OBSTACLE_DIST) / float(FREE_DIST - OBSTACLE_DIST)
                    speed = int(15 + ratio * (DRIVE_SPEED - 15))
 
                drive(speed, speed)
 
                # Balayage aller-retour entre -SCAN_MAX et +SCAN_MAX
                sweep_angle += SCAN_STEP * sweep_dir
                if sweep_angle >= SCAN_MAX:
                    sweep_angle = SCAN_MAX
                    sweep_dir   = -1
                elif sweep_angle <= -SCAN_MAX:
                    sweep_angle = -SCAN_MAX
                    sweep_dir   = 1
 
                sensor_goto(sweep_angle)
 
            time.sleep(0.05)
 
    except KeyboardInterrupt:
        stop()
        scan_motor.off(brake=True)
        sound.speak("Arret")
        print("Arrete.")
 
if __name__ == '__main__':
    main()