#!/usr/bin/env python3
"""
Analyse camera cote PC:
- detecte le marqueur ArUco du robot
- detecte les QR codes / marqueurs ArUco visibles
- detecte la balle rouge par filtre HSV + contours
- colle les deux images camera dans une seule image analysee
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
import struct
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

LABEL_COLOR = (255, 255, 255)
MARKER_COLOR = (0, 255, 0)
QR_COLOR = (255, 0, 255)
SEPARATOR_COLOR = (40, 40, 40)

UDP_HEADER_SIZE = 8
UDP_MESSAGE_HEADER_SIZE = 4


def marker_center(corners):
    pts = corners.reshape(4, 2)
    return pts.mean(axis=0)


def marker_angle_deg(corners):
    pts = corners.reshape(4, 2)
    top_mid = (pts[0] + pts[1]) / 2.0
    bottom_mid = (pts[2] + pts[3]) / 2.0
    direction = top_mid - bottom_mid
    return math.degrees(math.atan2(direction[1], direction[0]))


def polygon_center(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    return pts.mean(axis=0)


def polygon_angle_deg(points):
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 2:
        return 0.0
    direction = pts[1] - pts[0]
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


def detect_aruco_markers(frame, aruco_dict, parameters):
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    markers = []

    if ids is not None:
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            center = marker_center(marker_corners)
            markers.append({
                "id": int(marker_id),
                "center_px": (float(center[0]), float(center[1])),
                "angle_deg": float(marker_angle_deg(marker_corners)),
                "corners": marker_corners.reshape(4, 2).astype(float).tolist(),
            })

    return markers, corners, ids


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
                    "data": data,
                    "center_px": (float(center[0]), float(center[1])),
                    "angle_deg": float(polygon_angle_deg(qr_points)),
                    "corners": np.asarray(qr_points, dtype=float).reshape(-1, 2).tolist(),
                })
            return qrcodes

    data, points, _ = qr_detector.detectAndDecode(frame)
    if data and points is not None:
        center = polygon_center(points)
        qrcodes.append({
            "data": data,
            "center_px": (float(center[0]), float(center[1])),
            "angle_deg": float(polygon_angle_deg(points)),
            "corners": np.asarray(points, dtype=float).reshape(-1, 2).tolist(),
        })

    return qrcodes


def detect_state(frame, aruco_dict, parameters, qr_detector):
    state = {
        "ts": time.time(),
        "robot": None,
        "ball": None,
        "markers": [],
        "qrcodes": [],
    }

    markers, corners, ids = detect_aruco_markers(frame, aruco_dict, parameters)
    qrcodes = detect_qr_codes(frame, qr_detector)

    state["markers"] = [
        {
            "id": marker["id"],
            "x_cm": px_to_cm(marker["center_px"], frame.shape)["x_cm"],
            "y_cm": px_to_cm(marker["center_px"], frame.shape)["y_cm"],
            "angle_deg": marker["angle_deg"],
        }
        for marker in markers
    ]
    state["qrcodes"] = [
        {
            "data": qr["data"],
            "x_cm": px_to_cm(qr["center_px"], frame.shape)["x_cm"],
            "y_cm": px_to_cm(qr["center_px"], frame.shape)["y_cm"],
            "angle_deg": qr["angle_deg"],
        }
        for qr in qrcodes
    ]

    for marker in markers:
        if marker["id"] == ROBOT_MARKER_ID:
            pos = px_to_cm(marker["center_px"], frame.shape)
            pos["angle_deg"] = marker["angle_deg"]
            state["robot"] = pos

    return state, corners, ids, qrcodes


def annotate_qrcodes(frame, qrcodes):
    for qr in qrcodes:
        pts = np.asarray(qr["corners"], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(frame, [pts], True, QR_COLOR, 2)
        x, y = map(int, qr["center_px"])
        label = "QR {}".format(qr["data"])
        cv2.circle(frame, (x, y), 4, QR_COLOR, -1)
        cv2.putText(frame, label, (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, QR_COLOR, 2)


def put_camera_label(frame, label, origin):
    x, y = origin
    cv2.rectangle(frame, (x, y), (x + 92, y + 28), (0, 0, 0), -1)
    cv2.putText(frame, label, (x + 8, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, LABEL_COLOR, 2)


def compose_camera_frames(frame0, frame1, stack):
    if frame1 is None:
        frame = frame0.copy()
        put_camera_label(frame, "CAM 0", (8, 8))
        return frame

    frame1 = cv2.resize(frame1, (frame0.shape[1], frame0.shape[0]))

    if stack == "horizontal":
        separator = np.full((frame0.shape[0], 8, 3), SEPARATOR_COLOR, dtype=np.uint8)
        frame = np.hstack((frame0, separator, frame1))
        put_camera_label(frame, "CAM 0", (8, 8))
        put_camera_label(frame, "CAM 1", (frame0.shape[1] + separator.shape[1] + 8, 8))
        return frame

    separator = np.full((8, frame0.shape[1], 3), SEPARATOR_COLOR, dtype=np.uint8)
    frame = np.vstack((frame0, separator, frame1))
    put_camera_label(frame, "CAM 0", (8, 8))
    put_camera_label(frame, "CAM 1", (8, frame0.shape[0] + separator.shape[0] + 8))
    return frame


def open_camera(cam_id, width, height):
    camera = cv2.VideoCapture(cam_id)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return camera


class UdpFrameReceiver:
    """Recoit les images JPEG fragmentees comme dans receiver.py."""

    def __init__(self, ip, port, max_packet_size):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.max_packet_size = max_packet_size
        self.data_buffer = {}
        self.current_frame_id = -1

    def read(self):
        while True:
            packet, _ = self.sock.recvfrom(self.max_packet_size)
            if len(packet) < UDP_HEADER_SIZE:
                continue

            packet_id, frame_id = struct.unpack("II", packet[:UDP_HEADER_SIZE])
            payload = packet[UDP_HEADER_SIZE:]

            if frame_id != self.current_frame_id:
                frame = self.decode_current_frame(frame_id)
                self.data_buffer = {}
                self.current_frame_id = frame_id
                self.data_buffer[packet_id] = payload
                if frame is not None:
                    return frame
                continue

            self.data_buffer[packet_id] = payload

    def decode_current_frame(self, next_frame_id):
        if self.current_frame_id == -1:
            return None
        if self.current_frame_id + 1 != next_frame_id:
            return None
        if not self.data_buffer:
            return None

        full_data = b"".join(self.data_buffer[i] for i in sorted(self.data_buffer))
        frame_data = full_data[UDP_MESSAGE_HEADER_SIZE:]
        frame_buffer = np.frombuffer(frame_data, dtype=np.uint8)
        return cv2.imdecode(frame_buffer, 1)

    def close(self):
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description="Serveur vision robot ArUco/QR + balle rouge")
    parser.add_argument("--robot-ip", default="255.255.255.255", help="IP du robot ou broadcast")
    parser.add_argument("--port", type=int, default=8081, help="Port UDP JSON")
    parser.add_argument("--cam0", type=int, default=1)
    parser.add_argument("--cam1", type=int, default=2)
    parser.add_argument("--single-camera", action="store_true")
    parser.add_argument(
        "--udp-video",
        action="store_true",
        help="Recoit une frame JPEG UDP comme receiver.py au lieu d'ouvrir les cameras",
    )
    parser.add_argument("--udp-ip", default="", help="IP locale d'ecoute pour --udp-video")
    parser.add_argument("--udp-port", type=int, default=8080, help="Port video UDP pour --udp-video")
    parser.add_argument("--max-packet-size", type=int, default=1400)
    parser.add_argument(
        "--stack",
        choices=("vertical", "horizontal"),
        default="vertical",
        help="Comment coller les deux images camera",
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--scale-send", type=int, default=3)
    parser.add_argument("--ball-min-area", type=float, default=80.0)
    parser.add_argument("--show-mask", action="store_true", help="Affiche le masque rouge")
    args = parser.parse_args()

    frame_receiver = None
    camera0 = None
    camera1 = None
    if args.udp_video:
        frame_receiver = UdpFrameReceiver(args.udp_ip, args.udp_port, args.max_packet_size)
        print("Reception video UDP sur {}:{}...".format(args.udp_ip or "0.0.0.0", args.udp_port))
    else:
        camera0 = open_camera(args.cam0, args.width, args.height)
        camera1 = None if args.single_camera else open_camera(args.cam1, args.width, args.height)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()
    qr_detector = cv2.QRCodeDetector()

    while True:
        if frame_receiver is not None:
            frame = frame_receiver.read()
            if frame is None:
                continue
        else:
            ok0, frame0 = camera0.read()
            if not ok0:
                continue

            frame1 = None
            if camera1 is not None:
                ok1, frame1 = camera1.read()
                if not ok1:
                    frame1 = None

            frame = compose_camera_frames(frame0, frame1, args.stack)

        if args.scale_send > 1:
            frame = cv2.resize(
                frame,
                (frame.shape[1] // args.scale_send, frame.shape[0] // args.scale_send),
            )

        state, corners, ids, qrcodes = detect_state(frame, aruco_dict, parameters, qr_detector)
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
        annotate_qrcodes(frame, qrcodes)
        cv2.imshow("vision_server", frame)
        if args.show_mask:
            cv2.imshow("red_ball_mask", red_mask)

        ch = chr(cv2.waitKey(1) & 0xFF)
        if ch in ("q", "Q"):
            break

    if frame_receiver is not None:
        frame_receiver.close()
    if camera0 is not None:
        camera0.release()
    if camera1 is not None:
        camera1.release()
    sock.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
