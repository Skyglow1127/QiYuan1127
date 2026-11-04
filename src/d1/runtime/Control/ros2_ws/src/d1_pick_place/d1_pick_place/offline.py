"""Offline RGB-D tests / Camera snapshot replay / printable marker. No ROS I/O."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from .geometry import detect


def fixture():
    k = np.array([[600., 0, 320], [0, 600, 240], [0, 0, 1]])
    image = np.full((480, 640, 3), 230, np.uint8)
    depth = np.full((480, 640), .6, np.float32)
    code = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 23, 61)
    image[210:271, 370:431] = cv2.cvtColor(code, cv2.COLOR_GRAY2BGR)
    image[222:259, 242:279] = [0, 0, 255]
    depth[222:259, 242:279] = .565
    return image, depth, k, np.zeros(5)


def marker_svg(path, size=.06, marker_id=23):
    bits = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 6)
    # 10 mm quiet border. Printed black outer square = configured 60 mm.
    width_mm = size * 1000 + 20
    cells = ['<rect width="100%" height="100%" fill="white"/>']
    for y in range(6):
        for x in range(6):
            if bits[y, x] == 0:
                cells.append(f'<rect x="{10+x*size*1000/6}" y="{10+y*size*1000/6}" width="{size*1000/6}" height="{size*1000/6}" fill="black"/>')
    Path(path).write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_mm}mm" height="{width_mm}mm" viewBox="0 0 {width_mm} {width_mm}">' + ''.join(cells) + '</svg>')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--snapshot', help='Camera/output/last_session.yml; never produces live target')
    p.add_argument('--out', default='offline_result.json')
    p.add_argument('--marker', help='Generate 60 mm marker SVG for 100%% print scale')
    args = p.parse_args()
    if args.marker:
        marker_svg(args.marker)
        print(f'Wrote {args.marker}: ID 23, black square 60 mm; print at 100%, verify ruler measurement.')
        return
    if args.snapshot:
        path = Path(args.snapshot)
        f = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not f.isOpened():
            raise SystemExit('Cannot open snapshot')
        image = cv2.imread(str(path.parent / f.getNode('color').string()))
        raw = cv2.imread(str(path.parent / f.getNode('depth').string()), cv2.IMREAD_UNCHANGED)
        k, distortion = f.getNode('camera_matrix').mat(), f.getNode('distortion').mat()
        aligned, unit = f.getNode('aligned_to_color').real(), f.getNode('depth_unit_m').real()
        f.release()
        if image is None or raw is None or not aligned or not np.isfinite(unit) or unit <= 0:
            raise SystemExit('Snapshot RGB-D missing or not aligned')
        depth = raw.astype(np.float32) * unit
    else:
        image, depth, k, distortion = fixture()
    result = {'offline': True, 'motion_command_ready': False, 'source': args.snapshot or 'SYNTHETIC RGB-D'}
    try:
        cube, marker = detect(image, depth, k, distortion)
        result.update(detected=True, T_camera_cube=cube.tolist(), T_camera_marker=marker.tolist())
    except ValueError as exc:
        result.update(detected=False, reason=str(exc))
    Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False))
