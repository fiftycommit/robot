import math

import mock_strategie as commande


CHERCHER = "chercher balle"
SE_PLACER = "se placer"
APPROCHER = "approcher balle"
POUSSER = "pousser vers but"
EVITER = "eviter obstacle"
RECULER = "reculer"
SCAN_GAUCHE = "scan gauche"
SCAN_DROITE = "scan droite"
CHOISIR_PASSAGE = "choisir passage"

BUT_X = 291.0
BUT_Y = 195.0

etat_actuel = CHERCHER
but_courant = (BUT_X, BUT_Y)

ANGLE_OK = 8
DIST_CONTACT = 18
DIST_PLACEMENT = 32
DIST_POINT_OK = 10
OBSTACLE_DANGER = 35
DIST_TIR = 19
DETECTION_TIR = 38
ANGLE_DETECTION = 25
SCAN_ANGLE = 35
RECUL_FRAMES = 18
SCAN_FRAMES = 18
TURN_FRAMES = 20

mode_auto = None
mode_timer = 0
last_shot_check = False
score_gauche = 0
score_droite = 0
choix_direction = 1


def angle_vers(robot, cible):
    dx = cible["x_cm"] - robot["x_cm"]
    dy = cible["y_cm"] - robot["y_cm"]
    return math.degrees(math.atan2(dy, dx))


def ecart_angle(angle_cible, angle_robot):
    return (angle_cible - angle_robot + 180) % 360 - 180


def distance(a, b):
    return math.hypot(a["x_cm"] - b["x_cm"], a["y_cm"] - b["y_cm"])


def reset():
    global etat_actuel, but_courant, mode_auto, mode_timer
    global last_shot_check, score_gauche, score_droite, choix_direction

    etat_actuel = CHERCHER
    but_courant = (BUT_X, BUT_Y)
    mode_auto = None
    mode_timer = 0
    last_shot_check = False
    score_gauche = 0
    score_droite = 0
    choix_direction = 1


def point_derriere_balle(balle):
    dx = BUT_X - balle["x_cm"]
    dy = BUT_Y - balle["y_cm"]
    norme = math.hypot(dx, dy) or 1
    return {
        "x_cm": balle["x_cm"] - dx / norme * DIST_PLACEMENT,
        "y_cm": balle["y_cm"] - dy / norme * DIST_PLACEMENT,
    }


def obstacle_devant(robot, obstacle):
    d = distance(robot, obstacle)
    if d > OBSTACLE_DANGER:
        return False

    cible = angle_vers(robot, obstacle)
    return abs(ecart_angle(cible, robot["angle"])) < 35


def objet_detecte_devant(robot, objet):
    if distance(robot, objet) > DETECTION_TIR:
        return False
    cible = angle_vers(robot, objet)
    return abs(ecart_angle(cible, robot["angle"])) < ANGLE_DETECTION


def objets_devant(robot, balle, obstacle):
    objets = []
    for nom, objet in (("balle", balle), ("obstacle", obstacle)):
        if objet_detecte_devant(robot, objet):
            objets.append((distance(robot, objet), nom, objet))
    objets.sort(key=lambda item: item[0])
    return objets


def score_direction(robot, balle, obstacle, angle_offset):
    angle_base = robot["angle"] + angle_offset
    score = 999
    for objet in (balle, obstacle):
        cible = angle_vers(robot, objet)
        if abs(ecart_angle(cible, angle_base)) < ANGLE_DETECTION:
            score = min(score, distance(robot, objet))
    return score


def demarrer_recul():
    global mode_auto, mode_timer
    mode_auto = RECULER
    mode_timer = RECUL_FRAMES


def jouer_reprise(robot, balle, obstacle):
    global etat_actuel, mode_auto, mode_timer
    global score_gauche, score_droite, choix_direction

    if mode_auto == RECULER:
        etat_actuel = RECULER
        commande.recule(35)
        mode_timer -= 1
        if mode_timer <= 0:
            mode_auto = SCAN_GAUCHE
            mode_timer = SCAN_FRAMES
        return True

    if mode_auto == SCAN_GAUCHE:
        etat_actuel = SCAN_GAUCHE
        commande.tourneG(30)
        mode_timer -= 1
        if mode_timer <= 0:
            score_gauche = score_direction(robot, balle, obstacle, 0)
            mode_auto = SCAN_DROITE
            mode_timer = SCAN_FRAMES * 2
        return True

    if mode_auto == SCAN_DROITE:
        etat_actuel = SCAN_DROITE
        commande.tourneD(30)
        mode_timer -= 1
        if mode_timer <= 0:
            score_droite = score_direction(robot, balle, obstacle, 0)
            choix_direction = -1 if score_gauche >= score_droite else 1
            mode_auto = CHOISIR_PASSAGE
            mode_timer = TURN_FRAMES
        return True

    if mode_auto == CHOISIR_PASSAGE:
        etat_actuel = CHOISIR_PASSAGE
        if choix_direction < 0:
            commande.tourneG(30)
        else:
            commande.tourneD(30)
        mode_timer -= 1
        if mode_timer <= 0:
            mode_auto = None
        return True

    return False


def aller_vers(robot, cible, vitesse=50):
    diff = ecart_angle(angle_vers(robot, cible), robot["angle"])

    if abs(diff) > ANGLE_OK:
        if diff > 0:
            commande.tourneD(30)
        else:
            commande.tourneG(30)
    else:
        commande.avance(vitesse)


def jouer_tour(robot, balle, obstacle):
    global etat_actuel, but_courant, last_shot_check

    if jouer_reprise(robot, balle, obstacle):
        return

    if last_shot_check:
        last_shot_check = False
        if objets_devant(robot, balle, obstacle):
            demarrer_recul()
            etat_actuel = RECULER
            commande.recule(35)
            return

    if objet_detecte_devant(robot, balle) and distance(robot, balle) <= DIST_TIR:
        etat_actuel = POUSSER
        but_courant = (balle["x_cm"], balle["y_cm"])
        last_shot_check = True
        commande.tir()
        return

    objets = objets_devant(robot, balle, obstacle)
    if objets and objets[0][1] == "obstacle":
        if objets[0][0] <= DIST_TIR:
            etat_actuel = POUSSER
            last_shot_check = True
            commande.tir()
        else:
            etat_actuel = APPROCHER
            commande.avance(25)
        return

    if objet_detecte_devant(robot, balle):
        etat_actuel = APPROCHER
        but_courant = (balle["x_cm"], balle["y_cm"])
        commande.avance(25)
        return

    cible = point_derriere_balle(balle)
    dist_balle = distance(robot, balle)
    dist_cible = distance(robot, cible)

    if dist_cible > DIST_POINT_OK and dist_balle > DIST_CONTACT:
        but_courant = (cible["x_cm"], cible["y_cm"])
        etat_actuel = SE_PLACER
        aller_vers(robot, cible, vitesse=50)
        return

    if dist_balle > DIST_CONTACT:
        but_courant = (balle["x_cm"], balle["y_cm"])
        etat_actuel = APPROCHER
        aller_vers(robot, balle, vitesse=45)
        return

    etat_actuel = POUSSER
    but_courant = (BUT_X, BUT_Y)
    aller_vers(robot, {"x_cm": BUT_X, "y_cm": BUT_Y}, vitesse=45)
