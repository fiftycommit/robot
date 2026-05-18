#!/usr/bin/env python3

import pygame
import math
import sys
import mock_strategie as commande
import test_strategie as strategie

SCALE       = 1.5
ARENA_W_CM  = 301
ARENA_H_CM  = 390
ARENA_W_PX  = int(ARENA_W_CM * SCALE)
ARENA_H_PX  = int(ARENA_H_CM * SCALE)
OFFSET_X    = 40
OFFSET_Y    = 40
WIN_W       = ARENA_W_PX + OFFSET_X * 2 + 250
WIN_H       = ARENA_H_PX + OFFSET_Y * 2

FPS          = 30
VITESSE_SIM  = 40
MARGE_BORD_CM = 25

# Rayons en CM (pas en pixels !)
RAYON_ROBOT_CM   = 10
RAYON_BALLE_CM   = 7
RAYON_OBS_CM     = 12
DETECTION_TIR_CM = 38
ANGLE_DETECTION  = 25

def cm_to_px(x, y):
    return int(OFFSET_X + x * SCALE), int(OFFSET_Y + y * SCALE)

def px_to_cm(px, py):
    return (px - OFFSET_X) / SCALE, (py - OFFSET_Y) / SCALE

def clamp_balle():
    balle['x_cm'] = max(RAYON_BALLE_CM, min(ARENA_W_CM - RAYON_BALLE_CM, balle['x_cm']))
    balle['y_cm'] = max(RAYON_BALLE_CM, min(ARENA_H_CM - RAYON_BALLE_CM, balle['y_cm']))

# === État simulé ===
robot    = {'x_cm': 50.0,  'y_cm': 195.0, 'angle': 0.0}
balle    = {'x_cm': 150.0, 'y_cm': 195.0}
obstacle = {'x_cm': 200.0, 'y_cm': 150.0}

# Pour le drag & drop de l'obstacle
drag_obstacle = False

pygame.init()
screen = pygame.display.set_mode((WIN_W, WIN_H))
pygame.display.set_caption("Simulateur IA Robot")
clock  = pygame.time.Clock()
try:
    pygame.font.init()
    font = pygame.font.SysFont("monospace", 14)
except Exception as exc:
    font = None
    print(f"[simulateur] Texte desactive: pygame.font indisponible ({exc})")

def draw_text(txt, color, pos):
    if font is None:
        return
    screen.blit(font.render(txt, True, color), pos)

def draw_arena():
    screen.fill((30, 30, 30))
    pygame.draw.rect(screen, (60, 60, 60),
        (OFFSET_X, OFFSET_Y, ARENA_W_PX, ARENA_H_PX))
    pygame.draw.rect(screen, (200, 180, 50),
        (OFFSET_X, OFFSET_Y, ARENA_W_PX, ARENA_H_PX), 3)
    but_x, but_y = cm_to_px(281, 145)
    pygame.draw.rect(screen, (50, 200, 80),
        (but_x, but_y, int(20*SCALE), int(100*SCALE)), 0)
    pygame.draw.rect(screen, (50, 200, 80),
        (but_x, but_y, int(20*SCALE), int(100*SCALE)), 2)
    draw_text("BUT", (50, 200, 80), (but_x + 2, but_y + 40))

def draw_robot(r):
    px, py = cm_to_px(r['x_cm'], r['y_cm'])
    pygame.draw.circle(screen, (70, 130, 200), (px, py), int(RAYON_ROBOT_CM * SCALE))
    pygame.draw.circle(screen, (255, 255, 255), (px, py), int(RAYON_ROBOT_CM * SCALE), 2)
    rad = math.radians(r['angle'])
    ex = int(px + (RAYON_ROBOT_CM * SCALE + 8) * math.cos(rad))
    ey = int(py + (RAYON_ROBOT_CM * SCALE + 8) * math.sin(rad))
    pygame.draw.line(screen, (255, 80, 80), (px, py), (ex, ey), 3)

    for angle in (-ANGLE_DETECTION, ANGLE_DETECTION):
        cone_rad = math.radians(r['angle'] + angle)
        sx = int(px + DETECTION_TIR_CM * SCALE * math.cos(cone_rad))
        sy = int(py + DETECTION_TIR_CM * SCALE * math.sin(cone_rad))
        pygame.draw.line(screen, (240, 210, 80), (px, py), (sx, sy), 1)

def draw_balle(b):
    px, py = cm_to_px(b['x_cm'], b['y_cm'])
    pygame.draw.circle(screen, (255, 140, 0), (px, py), int(RAYON_BALLE_CM * SCALE))
    pygame.draw.circle(screen, (255, 255, 255), (px, py), int(RAYON_BALLE_CM * SCALE), 1)

def draw_obstacle(o):
    px, py = cm_to_px(o['x_cm'], o['y_cm'])
    r = int(RAYON_OBS_CM * SCALE)
    pygame.draw.rect(screen, (200, 60, 60), (px-r, py-r, r*2, r*2))
    pygame.draw.rect(screen, (255, 255, 255), (px-r, py-r, r*2, r*2), 2)
    draw_text("OBS", (255, 255, 255), (px - 12, py + r + 2))

def draw_infos(etat_cmd):
    panel_x = ARENA_W_PX + OFFSET_X * 2 + 10
    y = OFFSET_Y

    def line(txt, color=(220, 220, 220)):
        nonlocal y
        draw_text(txt, color, (panel_x, y))
        y += 20

    line("=== ETAT IA ===", (255, 220, 50))
    line(f"Etat : {strategie.etat_actuel}", (100, 200, 255))
    line(f"But  : {strategie.but_courant}", (100, 200, 255))
    line("")
    line("=== ROBOT ===", (255, 220, 50))
    line(f"x   = {robot['x_cm']:.1f} cm")
    line(f"y   = {robot['y_cm']:.1f} cm")
    line(f"ang = {robot['angle']:.1f} deg")
    line("")
    line("=== BALLE ===", (255, 220, 50))
    line(f"x   = {balle['x_cm']:.1f} cm")
    line(f"y   = {balle['y_cm']:.1f} cm")
    line("")
    line("=== OBSTACLE ===", (255, 220, 50))
    line(f"x   = {obstacle['x_cm']:.1f} cm")
    line(f"y   = {obstacle['y_cm']:.1f} cm")
    line("")
    line("=== COMMANDE ===", (255, 220, 50))
    action = etat_cmd['action']
    col = (100, 255, 100) if action == "avance" else \
          (255, 100, 100) if action == "recule" else \
          (255, 80, 255)  if action == "tir" else \
          (255, 180, 50)  if "tourne" in action else \
          (180, 180, 180)
    line(f"{action} v={etat_cmd['vitesse']}", col)
    line("")
    line("=== CONTROLES ===", (255, 220, 50))
    line("Clic gauche : deplace balle")
    line("Clic droit  : drag obstacle")
    line("R           : reset tout")
    line("Q           : quitter")

def update_robot(dt, etat_cmd):
    action  = etat_cmd['action']
    vitesse = etat_cmd['vitesse']
    spd     = VITESSE_SIM * (vitesse / 50) * dt

    # --- Déplacement robot ---
    if action == "avance":
        robot['x_cm'] += spd * math.cos(math.radians(robot['angle']))
        robot['y_cm'] += spd * math.sin(math.radians(robot['angle']))
    elif action == "recule":
        robot['x_cm'] -= spd * math.cos(math.radians(robot['angle']))
        robot['y_cm'] -= spd * math.sin(math.radians(robot['angle']))
    elif action == "tourneD":
        robot['angle'] += 60 * dt
    elif action == "tourneG":
        robot['angle'] -= 60 * dt
    elif action == "tir":
        rad = math.radians(robot['angle'])
        front_x = robot['x_cm'] + math.cos(rad) * (RAYON_ROBOT_CM + 8)
        front_y = robot['y_cm'] + math.sin(rad) * (RAYON_ROBOT_CM + 8)
        dx = balle['x_cm'] - front_x
        dy = balle['y_cm'] - front_y
        if math.sqrt(dx**2 + dy**2) < 30:
            balle['x_cm'] += math.cos(rad) * 45
            balle['y_cm'] += math.sin(rad) * 45

    robot['x_cm'] = max(MARGE_BORD_CM, min(ARENA_W_CM - MARGE_BORD_CM, robot['x_cm']))
    robot['y_cm'] = max(MARGE_BORD_CM, min(ARENA_H_CM - MARGE_BORD_CM, robot['y_cm']))
    robot['angle'] = (robot['angle'] + 180) % 360 - 180

    # --- Collision robot → balle ---
    dx   = balle['x_cm'] - robot['x_cm']
    dy   = balle['y_cm'] - robot['y_cm']
    dist = math.sqrt(dx**2 + dy**2)
    seuil_contact = RAYON_ROBOT_CM + RAYON_BALLE_CM + 2  # petite marge pour la pelle

    if action == "avance" and dist < seuil_contact and dist > 0:
        nx = dx / dist
        ny = dy / dist
        # Replace la balle au contact
        balle['x_cm'] = robot['x_cm'] + nx * seuil_contact
        balle['y_cm'] = robot['y_cm'] + ny * seuil_contact
        # Impulsion proportionnelle à la vitesse
        balle['x_cm'] += nx * spd * 1.5
        balle['y_cm'] += ny * spd * 1.5

    # --- Collision robot → obstacle (visuel seulement, pour ne pas bloquer le test IA) ---
    dx_o  = robot['x_cm'] - obstacle['x_cm']
    dy_o  = robot['y_cm'] - obstacle['y_cm']
    dist_o = math.sqrt(dx_o**2 + dy_o**2)
    seuil_obs = RAYON_ROBOT_CM + RAYON_OBS_CM

    if dist_o < seuil_obs and dist_o > 0:
        nx = dx_o / dist_o
        ny = dy_o / dist_o
        overlap = seuil_obs - dist_o
        robot['x_cm'] += nx * (overlap + 1)
        robot['y_cm'] += ny * (overlap + 1)

    # --- Bornage balle ---
    clamp_balle()

# === Boucle principale ===
running = True
drag_obstacle = False

while running:
    dt = clock.tick(FPS) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False
            if event.key == pygame.K_r:
                robot['x_cm']    = 50.0
                robot['y_cm']    = 195.0
                robot['angle']   = 0.0
                balle['x_cm']    = 150.0
                balle['y_cm']    = 195.0
                obstacle['x_cm'] = 200.0
                obstacle['y_cm'] = 150.0
                strategie.reset()

        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            cx, cy = px_to_cm(mx, my)
            if event.button == 1:   # clic gauche → balle
                balle['x_cm'] = cx
                balle['y_cm'] = cy
                clamp_balle()
                strategie.etat_actuel = strategie.SE_PLACER
            elif event.button == 3: # clic droit → commence drag obstacle
                drag_obstacle = True

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 3:
                drag_obstacle = False

        if event.type == pygame.MOUSEMOTION:
            if drag_obstacle:       # drag obstacle en temps réel
                mx, my = pygame.mouse.get_pos()
                cx, cy = px_to_cm(mx, my)
                obstacle['x_cm'] = max(0, min(ARENA_W_CM, cx))
                obstacle['y_cm'] = max(0, min(ARENA_H_CM, cy))

    # --- Appel IA ---
    strategie.jouer_tour(robot, balle, obstacle)
    etat_cmd = commande.get_etat()

    # --- Simulation physique ---
    update_robot(dt, etat_cmd)

    # --- Rendu ---
    draw_arena()
    draw_robot(robot)
    draw_balle(balle)
    draw_obstacle(obstacle)
    draw_infos(etat_cmd)
    pygame.display.flip()

pygame.quit()
sys.exit()
