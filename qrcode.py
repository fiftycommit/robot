import argparse

import cv2


def build_aruco(marker_id: int, output: str, size: int = 1600) -> None:
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    max_id = 99
    if not 0 <= marker_id <= max_id:
        raise ValueError(f"ID invalide: {marker_id}. Plage autorisee: 0..{max_id}")

    marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
    cv2.imwrite(output, marker)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Genere un marqueur ArUco PNG")
    parser.add_argument(
        "--id",
        type=int,
        default=67,
        help="ID du marqueur ArUco (0..99). Defaut: 67",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="marker_robot.png",
        help="Nom du fichier image de sortie (PNG)",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=1600,
        help="Taille de l'image du marqueur (pixels)",
    )
    args = parser.parse_args()

    build_aruco(args.id, args.output, args.size)
    print(f"Marqueur ArUco ID {args.id} sauvegarde : {args.output}")