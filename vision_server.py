#!/usr/bin/env python3
"""
Analyse camera cote PC:
- detecte le marqueur ArUco ID 67 du robot
- detecte la balle rouge ou bleue par filtre HSV
- calibration arene : clic sur les 4 coins au lancement
- calcule la position arene 0..255 par homographie
- affiche le flux annote
- diffuse les positions en UDP JSON vers le robot sur le port 8081

Detection:
  robot = ArUco ID 67
  balle = objet rouge ou bleu
"""

import argparse
import json
import math
import socket
import struct
import time

import cv2
import numpy as np


ROBOT_MARKER_ID = 67
ROBOT_MEMORY_SECONDS = 1.2
BALL_MEMORY_SECONDS = 0.8

ARENA_W_UNITS = 255.0
ARENA_H_UNITS = 255.0
ARENA_UNIT_NAME = "arena_0_255"

RED_LOWER_1 = np.array([0, 90, 70])
RED_UPPER_1 = np.array([12, 255, 255])
RED_LOWER_2 = np.array([170, 90, 70])
RED_UPPER_2 = np.array([179, 255, 255])

BLUE_LOWER = np.array([100, 80, 50])
BLUE_UPPER = np.array([130, 255, 255])

LABEL_COLOR   = (255, 255, 255)
QR_COLOR      = (255, 0, 255)
SEPARATOR_COLOR = (40, 40, 40)

UDP_HEADER_SIZE         = 8
UDP_MESSAGE_HEADER_SIZE = 4

CORNER_LABELS = ["Bas-Gauche", "Bas-Droit", "Haut-Droit", "Haut-Gauche"]
CORNER_COLORS = [(0, 255, 0), (0, 200, 255), (0, 0, 255), (255, 0, 255)]
# Coins arene correspondants (meme ordre).
# Repere mathematique normalise:
#   Bas-Gauche=(0,0), Bas-Droit=(255,0),
#   Haut-Droit=(255,255), Haut-Gauche=(0,255).
CORNER_ARENA = [
    (0.0, 0.0),
    (ARENA_W_UNITS, 0.0),
    (ARENA_W_UNITS, ARENA_H_UNITS),
    (0.0, ARENA_H_UNITS),
]


# ---------------------------------------------------------------------------
# Calibration par homographie pixel -> arene 0..255.
# L'axe Y OpenCV descend dans l'image, mais les destinations d'homographie
# placent le bas de l'arene a y=0 et le haut a y=255.
# ---------------------------------------------------------------------------

class ArenaCalibration:
    def __init__(self):
        self.pixel_points = []
        self.homography   = None

    def is_ready(self):
        return self.homography is not None

    def add_point(self, px):
        self.pixel_points.append(px)
        if len(self.pixel_points) == 4:
            src = np.array(self.pixel_points, dtype=np.float32)
            dst = np.array(CORNER_ARENA,      dtype=np.float32)
            self.homography, _ = cv2.findHomography(src, dst)

    def pixel_to_arena(self, px):
        if self.homography is None:
            return None
        pt     = np.array([[[float(px[0]), float(px[1])]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.homography)
        x = float(result[0][0][0])
        y = float(result[0][0][1])
        return arena_position(x, y)

    def calibration_errors(self):
        if self.homography is None:
            return []
        errors = []
        for px, expected in zip(self.pixel_points, CORNER_ARENA):
            mapped = self.pixel_to_arena(px)
            if mapped is None:
                continue
            err = math.hypot(mapped["x"] - expected[0], mapped["y"] - expected[1])
            errors.append((px, expected, mapped, err))
        return errors

    def clicked_area_px(self):
        if len(self.pixel_points) != 4:
            return 0.0
        pts = np.array(self.pixel_points, dtype=np.float32)
        return abs(float(cv2.contourArea(pts)))


class CalibrationState:
    def __init__(self, frame):
        self.display = frame.copy()
        self.clicks  = []
        self.done    = False

    def mouse_callback(self, event, x, y, flags, param):
        if self.done or event != cv2.EVENT_LBUTTONDOWN:
            return
        idx = len(self.clicks)
        self.clicks.append((x, y))
        color = CORNER_COLORS[idx]
        cv2.circle(self.display, (x, y), 8, color, -1)
        cv2.putText(self.display, CORNER_LABELS[idx], (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        print(f"  {idx+1}/4 {CORNER_LABELS[idx]} -> pixel ({x}, {y})")
        if len(self.clicks) == 4:
            self.done = True


def run_calibration(first_frame):
    calib = ArenaCalibration()
    state = CalibrationState(first_frame)

    cv2.namedWindow("calibration")
    cv2.setMouseCallback("calibration", state.mouse_callback)

    print("\n=== CALIBRATION ARENE ===")
    print("Cliquez les 4 coins du ruban jaune dans cet ordre :")
    for i, label in enumerate(CORNER_LABELS):
        print(f"  {i+1}. {label}")
    print("Appuyez sur Q pour annuler.\n")

    while not state.done:
        overlay = state.display.copy()
        idx = len(state.clicks)
        if idx < 4:
            msg = f"Cliquez : {CORNER_LABELS[idx]}  ({idx+1}/4)"
            cv2.putText(overlay, msg, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, CORNER_COLORS[idx], 2)
        cv2.imshow("calibration", overlay)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            cv2.destroyWindow("calibration")
            return None

    cv2.destroyWindow("calibration")

    for px in state.clicks:
        calib.add_point(px)

    print(f"Calibration OK  repere arene 0..{ARENA_W_UNITS:.0f} x 0..{ARENA_H_UNITS:.0f}")
    area = calib.clicked_area_px()
    print("Surface cliquee: {:.0f} px2".format(area))
    if area < 1000:
        print("  ATTENTION: surface tres faible, les coins sont probablement mal cliques.")
    print("Controle homographie pixel -> arene:")
    max_err = 0.0
    for i, (_, expected, mapped, err) in enumerate(calib.calibration_errors()):
        max_err = max(max_err, err)
        print(
            "  {}: attendu=({:.1f},{:.1f}) obtenu=({:.1f},{:.1f}) err={:.2f}".format(
                CORNER_LABELS[i], expected[0], expected[1], mapped["x"], mapped["y"], err
            )
        )
    if max_err > 2.0:
        print("  ATTENTION: erreur de calibration elevee, verifier l'ordre des clics.")
    print("")
    return calib


# ---------------------------------------------------------------------------
# Detection ArUco
# ---------------------------------------------------------------------------

def marker_center(corners):
    return corners.reshape(4, 2).mean(axis=0)

def marker_angle_deg(corners):
    pts       = corners.reshape(4, 2)
    top_mid   = (pts[0] + pts[1]) / 2.0
    bot_mid   = (pts[2] + pts[3]) / 2.0
    direction = top_mid - bot_mid
    # OpenCV a y vers le bas. On inverse dy pour publier un angle en repere
    # mathematique: 0 deg vers +X, 90 deg vers +Y.
    return math.degrees(math.atan2(-direction[1], direction[0]))

def aruco_frame_variants(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    contrast = clahe.apply(gray)
    blur = cv2.GaussianBlur(contrast, (0, 0), 1.0)
    sharp = cv2.addWeighted(contrast, 1.6, blur, -0.6, 0)
    variants = [gray, contrast, sharp]
    enlarged = [
        cv2.resize(image, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        for image in variants
    ]
    return [(image, 1.0) for image in variants] + [(image, 2.0) for image in enlarged]

def detect_aruco_markers(frame, aruco_dict, parameters, robust=False):
    detector        = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    best_corners    = []
    best_ids        = None

    if robust:
        candidates = aruco_frame_variants(frame)
    else:
        candidates = [(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 1.0)]

    for candidate, scale in candidates:
        corners, ids, _ = detector.detectMarkers(candidate)
        if ids is None:
            continue
        if scale != 1.0:
            corners = [corner / scale for corner in corners]
        if ROBOT_MARKER_ID in ids.flatten():
            best_corners = corners
            best_ids = ids
            break
        if best_ids is None or len(ids) > len(best_ids):
            best_corners = corners
            best_ids = ids

    markers = []
    corners = best_corners
    ids = best_ids
    if ids is not None:
        for mc, mid in zip(corners, ids.flatten()):
            c = marker_center(mc)
            markers.append({
                "id":        int(mid),
                "center_px": (float(c[0]), float(c[1])),
                "angle_deg": float(marker_angle_deg(mc)),
                "corners":   mc.reshape(4, 2).astype(float).tolist(),
            })
    return markers, corners, ids


# ---------------------------------------------------------------------------
# Detection QR
# ---------------------------------------------------------------------------

def polygon_center(points):
    return np.asarray(points, dtype=np.float32).reshape(-1, 2).mean(axis=0)

def polygon_angle_deg(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 2:
        return 0.0
    d = pts[1] - pts[0]
    return math.degrees(math.atan2(d[1], d[0]))

def detect_qr_codes(frame, qr_detector):
    qrcodes = []
    if hasattr(qr_detector, "detectAndDecodeMulti"):
        ok, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(frame)
        if ok and points is not None:
            for data, qr_points in zip(decoded_info, points):
                if not data:
                    continue
                center = polygon_center(qr_points)
                qrcodes.append({
                    "data":      data,
                    "center_px": (float(center[0]), float(center[1])),
                    "angle_deg": float(polygon_angle_deg(qr_points)),
                    "corners":   np.asarray(qr_points, dtype=float).reshape(-1, 2).tolist(),
                })
            return qrcodes
    data, points, _ = qr_detector.detectAndDecode(frame)
    if data and points is not None:
        center = polygon_center(points)
        qrcodes.append({
            "data":      data,
            "center_px": (float(center[0]), float(center[1])),
            "angle_deg": float(polygon_angle_deg(points)),
            "corners":   np.asarray(points, dtype=float).reshape(-1, 2).tolist(),
        })
    return qrcodes


# ---------------------------------------------------------------------------
# Detection balle rouge / bleue
# ---------------------------------------------------------------------------

def detect_ball(frame, min_area):
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m1     = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
    m2     = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    red_m  = cv2.bitwise_or(m1, m2)
    blue_m = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    kernel    = np.ones((3, 3), np.uint8)
    best      = None
    best_mask = red_m

    for mask, color_name in [(red_m, "red"), (blue_m, "blue")]:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(contour)
        if area < min_area:
            continue
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius <= 0:
            continue
        if area / (math.pi * radius * radius) < 0.25:
            continue
        if best is None or area > best["area_px"]:
            best = {
                "center_px": (float(x), float(y)),
                "radius_px": float(radius),
                "area_px":   float(area),
                "color":     color_name,
            }
            best_mask = mask

    return best, best_mask


# ---------------------------------------------------------------------------
# Construction etat complet
# ---------------------------------------------------------------------------

def arena_position(x, y):
    # Champs x/y propres + anciens alias pour rester compatible avec les
    # scripts EV3 existants. Ces valeurs ne sont pas des centimetres:
    # ce sont des unites normalisees dans le repere mathematique de l'arene.
    return {
        "x": x,
        "y": y,
        "unit": ARENA_UNIT_NAME,
        "arena_w": ARENA_W_UNITS,
        "arena_h": ARENA_H_UNITS,
        "x_arena": x,
        "y_arena": y,
        "x_gps": x,
        "y_gps": y,
        "x_cm": x,
        "y_cm": y,
    }

def px_to_arena_simple(point, frame_shape):
    h, w   = frame_shape[:2]
    x_px, y_px = point
    x = float(x_px / max(w, 1) * ARENA_W_UNITS)
    y = float((1.0 - y_px / max(h, 1)) * ARENA_H_UNITS)
    return arena_position(x, y)

def point_to_arena(point, frame_shape, calib):
    if calib and calib.is_ready():
        pos = calib.pixel_to_arena(point)
        if pos:
            return pos
    return px_to_arena_simple(point, frame_shape)

def build_state(frame, aruco_dict, parameters, qr_detector, calib,
                object_memory, enable_qr=False, robust_aruco=False):
    state = {
        "ts": time.time(),
        "unit": ARENA_UNIT_NAME,
        "arena": {"w": ARENA_W_UNITS, "h": ARENA_H_UNITS},
        "robot": None,
        "ball": None,
        "markers": [],
        "qrcodes": [],
    }

    markers, corners, ids = detect_aruco_markers(frame, aruco_dict, parameters, robust_aruco)
    qrcodes               = detect_qr_codes(frame, qr_detector) if enable_qr else []

    for m in markers:
        pos = point_to_arena(m["center_px"], frame.shape, calib)
        entry = {
            "id": m["id"],
            "source": "aruco",
            "center_px": m["center_px"],
            "angle_deg": m["angle_deg"],
            "angle_available": True,
            **pos
        }
        state["markers"].append(entry)
        if m["id"] == ROBOT_MARKER_ID:
            state["robot"] = {**entry}
            state["robot"]["stale"] = False
            object_memory["robot"] = {**state["robot"]}
            object_memory["robot_ts"] = state["ts"]

    if state["robot"] is None and object_memory.get("robot"):
        age = state["ts"] - object_memory.get("robot_ts", 0)
        if age <= ROBOT_MEMORY_SECONDS:
            state["robot"] = {**object_memory["robot"]}
            state["robot"]["stale"] = True
            state["robot"]["age"] = age

    for qr in qrcodes:
        pos = point_to_arena(qr["center_px"], frame.shape, calib)
        state["qrcodes"].append({
            "data": qr["data"],
            "center_px": qr["center_px"],
            "angle_deg": qr["angle_deg"],
            **pos
        })

    return state, corners, ids, qrcodes


# ---------------------------------------------------------------------------
# Annotations
# ---------------------------------------------------------------------------

def annotate_frame(frame, state, ball, calib, corners, ids, qrcodes):
    # Marqueurs ArUco
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Robot
    if state["robot"]:
        r = state["robot"]
        label = "ROBOT"
        color = (0, 255, 0)
        if r.get("stale"):
            label = "ROBOT memoire"
            color = (0, 180, 255)
        cv2.putText(frame, f"{label}  {r['x']:.1f},{r['y']:.1f} arena  {r['angle_deg']:.1f}deg",
                    (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
        if r.get("center_px"):
            x, y = map(int, r["center_px"])
            cv2.circle(frame, (x, y), 6, color, -1)
            angle = math.radians(r["angle_deg"])
            tip = (
                int(x + math.cos(angle) * 38),
                int(y - math.sin(angle) * 38)
            )
            cv2.arrowedLine(frame, (x, y), tip, color, 3, tipLength=0.35)
            cv2.putText(frame, label,
                        (x + 10, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(frame, f"{r['x']:.1f}, {r['y']:.1f} arena",
                        (x + 10, y - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    # QR codes
    for qr in qrcodes:
        pts = np.asarray(qr["corners"], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(frame, [pts], True, QR_COLOR, 2)
        x, y = map(int, qr["center_px"])
        cv2.circle(frame, (x, y), 4, QR_COLOR, -1)
        cv2.putText(frame, "QR " + qr["data"], (x + 8, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, QR_COLOR, 2)

    # Balle
    if ball:
        x, y   = map(int, ball["center_px"])
        radius = int(ball["radius_px"])
        color  = (0, 0, 255) if ball["color"] == "red" else (255, 100, 0)
        cv2.circle(frame, (x, y), radius, color, 2)
        cv2.circle(frame, (x, y), 3,      color, -1)
        cv2.putText(frame, f"BALL ({ball['color']})",
                    (x + 8, y - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        if state["ball"] and state["ball"].get("pos"):
            p = state["ball"]["pos"]
            cv2.putText(frame, f"{p['x']:.1f}, {p['y']:.1f} arena",
                        (x + 8, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

    # Contour arene calibree
    if calib and calib.is_ready():
        pts = np.array([tuple(map(int, p)) for p in calib.pixel_points], dtype=np.int32)
        cv2.polylines(frame, [pts], True, (0, 255, 255), 2)
        for i, (pt, col) in enumerate(zip(calib.pixel_points, CORNER_COLORS)):
            cv2.circle(frame, (int(pt[0]), int(pt[1])), 6, col, -1)
            cv2.putText(frame, CORNER_LABELS[i], (int(pt[0]) + 8, int(pt[1]) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)


# ---------------------------------------------------------------------------
# Camera / UDP
# ---------------------------------------------------------------------------

def put_camera_label(frame, label, origin):
    x, y = origin
    cv2.rectangle(frame, (x, y), (x + 92, y + 28), (0, 0, 0), -1)
    cv2.putText(frame, label, (x + 8, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, LABEL_COLOR, 2)

def compose_camera_frames(frame0, frame1, stack):
    if frame1 is None:
        f = frame0.copy(); put_camera_label(f, "CAM 0", (8, 8)); return f
    frame1 = cv2.resize(frame1, (frame0.shape[1], frame0.shape[0]))
    if stack == "horizontal":
        sep   = np.full((frame0.shape[0], 8, 3), SEPARATOR_COLOR, dtype=np.uint8)
        frame = np.hstack((frame0, sep, frame1))
        put_camera_label(frame, "CAM 0", (8, 8))
        put_camera_label(frame, "CAM 1", (frame0.shape[1] + 16, 8))
        return frame
    sep   = np.full((8, frame0.shape[1], 3), SEPARATOR_COLOR, dtype=np.uint8)
    frame = np.vstack((frame0, sep, frame1))
    put_camera_label(frame, "CAM 0", (8, 8))
    put_camera_label(frame, "CAM 1", (8, frame0.shape[0] + 16))
    return frame

def parse_camera_source(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return value

def open_camera(cam_source, width, height):
    cam = cv2.VideoCapture(cam_source)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv2.CAP_PROP_FPS, 30)
    return cam


class UdpFrameReceiver:
    def __init__(self, ip, port, max_packet_size):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.max_packet_size  = max_packet_size
        self.data_buffer      = {}
        self.current_frame_id = -1

    def read(self):
        while True:
            packet, _ = self.sock.recvfrom(self.max_packet_size)
            if len(packet) < UDP_HEADER_SIZE:
                continue
            packet_id, frame_id = struct.unpack("II", packet[:UDP_HEADER_SIZE])
            payload = packet[UDP_HEADER_SIZE:]
            if frame_id != self.current_frame_id:
                frame = self._decode()
                self.data_buffer      = {}
                self.current_frame_id = frame_id
                self.data_buffer[packet_id] = payload
                if frame is not None:
                    return frame
                continue
            self.data_buffer[packet_id] = payload

    def _decode(self):
        if self.current_frame_id == -1:
            return None
        if not self.data_buffer:
            return None
        full_data   = b"".join(self.data_buffer[i] for i in sorted(self.data_buffer))
        frame_data  = full_data[UDP_MESSAGE_HEADER_SIZE:]
        frame_buf   = np.frombuffer(frame_data, dtype=np.uint8)
        return cv2.imdecode(frame_buf, 1)

    def close(self):
        self.sock.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Vision robot ArUco ID67 + balle rouge/bleue")
    parser.add_argument("--robot-ip",       default="255.255.255.255")
    parser.add_argument("--port",           type=int,   default=8081)
    parser.add_argument("--cam0",           default="1",
                        help="Index camera OpenCV ou URL RTSP/HTTP de camera reseau")
    parser.add_argument("--cam1",           default="2",
                        help="Index camera OpenCV ou URL RTSP/HTTP de camera reseau")
    parser.add_argument("--single-camera",  action="store_true")
    parser.add_argument("--udp-video",      action="store_true")
    parser.add_argument("--udp-ip",         default="")
    parser.add_argument("--udp-port",       type=int,   default=8080)
    parser.add_argument("--max-packet-size",type=int,   default=1400)
    parser.add_argument("--stack",          choices=("vertical","horizontal"), default="vertical")
    parser.add_argument("--width",          type=int,   default=1280)
    parser.add_argument("--height",         type=int,   default=720)
    parser.add_argument("--scale-send",     type=int,   default=2)
    parser.add_argument("--ball-min-area",  type=float, default=20.0)
    parser.add_argument("--show-mask",      action="store_true")
    parser.add_argument("--enable-qr",      action="store_true")
    parser.add_argument("--fast-aruco",     action="store_true",
                        help="Desactive les essais de detection ArUco renforces")
    parser.add_argument("--robust-aruco",   action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-calib",       action="store_true",
                        help="Demarre sans calibration (GPS estime par largeur/hauteur image)")
    args = parser.parse_args()

    # Source video
    frame_receiver = None
    camera0 = camera1 = None
    if args.udp_video:
        frame_receiver = UdpFrameReceiver(args.udp_ip, args.udp_port, args.max_packet_size)
        print(f"Reception video UDP sur {args.udp_ip or '0.0.0.0'}:{args.udp_port}...")
    else:
        camera0 = open_camera(parse_camera_source(args.cam0), args.width, args.height)
        camera1 = None if args.single_camera else open_camera(parse_camera_source(args.cam1), args.width, args.height)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    aruco_dict  = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.008
    parameters.minMarkerPerimeterRate  = 0.02   # détecte les petits marqueurs (défaut 0.03)
    parameters.minMarkerPerimeterRate = 0.008
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 101
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    qr_detector = cv2.QRCodeDetector()
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.adaptiveThreshWinSizeMax = 101
    parameters.polygonalApproxAccuracyRate = 0.08
    parameters.minCornerDistanceRate = 0.02
    parameters.minDistanceToBorder = 2
    parameters.errorCorrectionRate = 0.8
    object_memory = {}

    def get_frame():
        if frame_receiver is not None:
            return frame_receiver.read()
        ok0, f0 = camera0.read()
        if not ok0:
            return None
        f1 = None
        if camera1 is not None:
            ok1, f1 = camera1.read()
            if not ok1:
                f1 = None
        return compose_camera_frames(f0, f1, args.stack)

    def scale(frame):
        if args.scale_send > 1:
            return cv2.resize(frame,
                (frame.shape[1] // args.scale_send, frame.shape[0] // args.scale_send))
        return frame

    # Calibration
    calib = None
    if not args.no_calib:
        print("Attente de la premiere frame...")
        first_frame = None
        while first_frame is None:
            first_frame = get_frame()
        calib = run_calibration(scale(first_frame))
        if calib is None:
            print("Calibration annulee, GPS estime par largeur/hauteur image.")

    # Boucle principale
    while True:
        frame = get_frame()
        if frame is None:
            continue
        frame = scale(frame)

        state, corners, ids, qrcodes = build_state(
            frame,
            aruco_dict,
            parameters,
            qr_detector,
            calib,
            object_memory,
            args.enable_qr,
            not args.fast_aruco
        )
        ball, ball_mask              = detect_ball(frame, args.ball_min_area)

        if ball:
            pos = point_to_arena(ball["center_px"], frame.shape, calib)
            state["ball"] = {
                **pos,
                "center_px": ball["center_px"],
                "pos":       pos,
                "pos_cm":    pos,
                "radius_px": ball["radius_px"],
                "color":     ball["color"],
                "stale":     False,
            }
            object_memory["ball"] = {**state["ball"]}
            object_memory["ball_ts"] = state["ts"]
        elif object_memory.get("ball"):
            age = state["ts"] - object_memory.get("ball_ts", 0)
            if age <= BALL_MEMORY_SECONDS:
                state["ball"] = {**object_memory["ball"]}
                state["ball"]["stale"] = True
                state["ball"]["age"] = age

        annotate_frame(frame, state, ball, calib, corners, ids, qrcodes)

        sock.sendto(json.dumps(state).encode("utf-8"), (args.robot_ip, args.port))

        cv2.imshow("vision_server", frame)
        if args.show_mask:
            cv2.imshow("ball_mask", ball_mask)

        if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q')):
            break

    if frame_receiver: frame_receiver.close()
    if camera0:        camera0.release()
    if camera1:        camera1.release()
    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
