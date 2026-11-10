#!/usr/bin/env python3
"""Generate the project's printable ChArUco board at an explicit physical size."""
import argparse
from pathlib import Path

import cv2
from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--squares-x", type=int, default=8)
    parser.add_argument("--squares-y", type=int, default=6)
    parser.add_argument("--square-mm", type=float, default=20)
    parser.add_argument("--marker-ratio", type=float, default=.72)
    parser.add_argument("--dpi", type=int, default=300)
    args = parser.parse_args()
    if not 4 <= args.squares_x <= 20 or not 4 <= args.squares_y <= 20:
        parser.error("squares must be 4..20")
    if not 5 <= args.square_mm <= 80 or not .5 <= args.marker_ratio <= .85:
        parser.error("invalid physical board dimensions")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(args.squares_x, args.squares_y, 1.0,
                                         args.marker_ratio, dictionary)
    width_mm = args.squares_x * args.square_mm
    height_mm = args.squares_y * args.square_mm
    width_px = round(width_mm / 25.4 * args.dpi)
    height_px = round(height_mm / 25.4 * args.dpi)
    bitmap = board.draw((width_px, height_px), marginSize=0, borderBits=1)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(bitmap).save(args.output, dpi=(args.dpi, args.dpi))
    print(f"{args.output}: {width_mm:g} x {height_mm:g} mm at {args.dpi} dpi")
    print("Print at 100% / actual size, disable fit-to-page, then measure square and marker sizes.")


if __name__ == "__main__":
    main()
