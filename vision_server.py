#!/usr/bin/env python3
"""
Analyse camera cote PC:
- detecte le marqueur ArUco du robot
- detecte la balle rouge par filtre HSV + contours
- affiche le flux annote
- diffuse les positions en UDP JSON vers le robot

Detection:
  robot = ID 67
  balle = petit objet rouge
"""

import argparse
import json
import math
import socket
import time

import cv2
import numpy as np


ROBOT_MARKER_ID = 67

ARENA_W_CM = 301.0
ARENA_H_CM = 390.0

RED_LOWER_1 = np.array([0, 90, 70])
RED_UPPER_1 = np.array([12, 255, 255])
RED_LOWER_2 = np.array([170, 90, 70])
RED_UPPER_2 = np.array([179, 255, 255])


def marker_center(corners):
    pts = corners.reshape(4, 2)
    return pts.mean(axis=0)


def marker_angle_deg(corners):
    pts = corners.reshape(4, 2)
    top_mid = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[2] + pts[3]) / 2.0
    direction = top_mid - bottom_mid
    return math.degrees(math.atan2(direction[1], direction[0]))


def px_to_cm(point, frame_shape):
    h, w = frame_shape[:2]
    x_px, y_px = point
    return {
        "x_cm": float(x_px / max(w, 1) * ARENA_W_CM),
        "y_cm": float(y_px / max(h, 1) * ARENA_H_CM),
    }


def detect_red_ball(frame, min_area):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, RED_LOWER_1, RED_UPPER_1)
    mask2 = cv2.inRange(hsv, RED_LOWER_2, RED_UPPER_2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None, mask

    (x, y), radius = cv2.minEnclosingCircle(contour)
    if radius <= 0:
        return None, mask

    circle_area = math.pi * radius * radius
    fill_ratio = area / circle_area if circle_area else 0
    if fill_ratio < 0.35:
        return None, mask

    return {
        "center_px": (float(x), float(y)),
        "radius_px": float(radius),
        "area_px": float(area),
    }, mask


def detect_state(frame, aruco_dict, parameters):
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    state = {"ts": time.time(), "robot": None, "ball": None}

    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            center = marker_center(marker_corners)
            pos = px_to_cm(center, frame.shape)

            if marker_id == ROBOT_MARKER_ID:
                pos["angle_deg"] = float(marker_angle_deg(marker_corners))
                state["robot"] = pos

    return state, corners, ids


def open_camera(cam_id, width, height):
    camera = cv2.VideoCapture(cam_id)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return camera


def main():
    parser = argparse.ArgumentParser(description="Serveur vision robot ArUco + balle rouge")
    parser.add_argument("--robot-ip", default="255.255.255.255", help="IP du robot ou broadcast")
    parser.add_argument("--port", type=int, default=8081, help="Port UDP JSON")
    parser.add_argument("--cam0", type=int, default=1)
    parser.add_argument("--cam1", type=int, default=2)
    parser.add_argument("--single-camera", action="store_true")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--scale-send", type=int, default=3)
    parser.add_argument("--ball-min-area", type=float, default=80.0)
    parser.add_argument("--show-mask", action="store_true", help="Affiche le masque rouge")
    args = parser.parse_args()

    camera0 = open_camera(args.cam0, args.width, args.height)
    camera1 = None if args.single_camera else open_camera(args.cam1, args.width, args.height)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()

    while True:
        ok0, frame0 = camera0.read()
        if not ok0:
            continue

        if camera1 is not None:
            ok1, frame1 = camera1.read()
            if ok1:
                frame1 = cv2.resize(frame1, (frame0.shape[1], frame0.shape[0]))
                frame = np.vstack((frame0, frame1))
            else:
                frame = frame0
        else:
            frame = frame0

        if args.scale_send > 1:
            frame = cv2.resize(
                frame,
                (frame.shape[1] // args.scale_send, frame.shape[0] // args.scale_send),
            )

        state, corners, ids = detect_state(frame, aruco_dict, parameters)
        ball, red_mask = detect_red_ball(frame, args.ball_min_area)
        if ball:
            state["ball"] = px_to_cm(ball["center_px"], frame.shape)
            state["ball"]["radius_px"] = ball["radius_px"]
            x, y = map(int, ball["center_px"])
            radius = int(ball["radius_px"])
            cv2.circle(frame, (x, y), radius, (0, 255, 255), 2)
            cv2.circle(frame, (x, y), 3, (0, 255, 255), -1)
            cv2.putText(
                frame,
                "BALL",
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

        payload = json.dumps(state).encode("utf-8")
        sock.sendto(payload, (args.robot_ip, args.port))

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("vision_server", frame)
        if args.show_mask:
            cv2.imshow("red_ball_mask", red_mask)

        ch = chr(cv2.waitKey(1) & 0xFF)
        if ch in ("q", "Q"):
            break

    camera0.release()
    if camera1 is not None:
        camera1.release()
    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
