#!/usr/bin/env python3
"""Offline chessboard corner/orientation preview; no robot or camera access."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board', required=True, type=Path)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    board = json.loads(args.board.read_text())
    if board.get('type', 'chessboard') != 'chessboard':
        parser.error('This preview supports ordinary chessboards only')
    cols, rows = board['inner_columns'], board['inner_rows']
    if type(cols) is not int or type(rows) is not int or not 3 <= cols <= 30 or not 3 <= rows <= 30:
        parser.error('Invalid inner corner dimensions')
    image = cv2.imread(str(args.image))
    if image is None:
        parser.error('Cannot read input image')
    found, corners = cv2.findChessboardCornersSB(
        image, (cols, rows), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'image': str(args.image.resolve()), 'board': str(args.board.resolve()),
              'image_sha256': hashlib.sha256(args.image.read_bytes()).hexdigest(),
              'board_sha256': hashlib.sha256(args.board.read_bytes()).hexdigest(),
              'inner_columns': cols, 'inner_rows': rows, 'detected': bool(found),
              'corner_count': len(corners) if found else 0,
              'physical_pitch_verified': False, 'physical_orientation_verified': False,
              'robot_motion_commands_sent': False,
              'note': 'O / +X / +Y show detector ordering only. Compare O with the same physical corner in every photo; this does not disambiguate a symmetric board automatically.'}
    if found:
        cv2.drawChessboardCorners(image, (cols, rows), corners, found)
        points = corners[:, 0, :]
        origin, x, y = [tuple(int(round(v)) for v in points[i]) for i in (0, cols-1, (rows-1)*cols)]
        cv2.arrowedLine(image, origin, x, (0, 0, 255), 2, tipLength=.08)
        cv2.arrowedLine(image, origin, y, (255, 0, 0), 2, tipLength=.08)
        for label, point, color in [('O', origin, (0, 160, 0)), ('+X', x, (0, 0, 255)), ('+Y', y, (255, 0, 0))]:
            cv2.putText(image, label, (point[0]+5, point[1]-8), cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2, cv2.LINE_AA)
        report['origin_pixel'] = points[0].tolist()
        report['corners_pixel'] = points.tolist()
    if not cv2.imwrite(str(args.output/'corners.png'), image):
        raise RuntimeError('Cannot save preview')
    (args.output/'check.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'corners_pixel'}, indent=2))
    return 0 if found else 1


if __name__ == '__main__':
    raise SystemExit(main())
