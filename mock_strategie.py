etat = {
    "action": "stop",
    "vitesse": 0,
}


def avance(vitesse=50):
    etat["action"] = "avance"
    etat["vitesse"] = vitesse


def recule(vitesse=35):
    etat["action"] = "recule"
    etat["vitesse"] = vitesse


def tourneG(vitesse=30):
    etat["action"] = "tourneG"
    etat["vitesse"] = vitesse


def tourneD(vitesse=30):
    etat["action"] = "tourneD"
    etat["vitesse"] = vitesse


def stop():
    etat["action"] = "stop"
    etat["vitesse"] = 0


def tir(vitesse=100):
    etat["action"] = "tir"
    etat["vitesse"] = vitesse


def get_etat():
    return dict(etat)
