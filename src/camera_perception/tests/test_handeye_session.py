"""End-to-end ChArUco hand-eye fixture plus motion-gated snapshot checks."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("aipc_handeye", ROOT / "scripts/d1_handeye_aipc.py")
SESSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SESSION)


def transform(rvec, translation):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = cv2.Rodrigues(np.asarray(rvec, np.float64))[0]
    result[:3, 3] = translation
    return result


class HandEyeSessionTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.CharucoBoard_create(8, 6, .02, .0144, self.dictionary)
        self.board_image = self.board.draw((1600, 1200), marginSize=0, borderBits=1)
        self.k = np.array([[700., 0, 320.], [0, 700., 240.], [0, 0, 1.]])
        self.distortion = np.zeros((1, 8), np.float64)
        self.truth = transform([.12, -.08, .05], [.035, -.02, .075])
        self.fixed_target = transform([.05, .03, -.02], [.42, -.12, .55])

    def tearDown(self):
        self.temporary.cleanup()

    def snapshot(self, index, camera_target, moving=False):
        directory = self.root / f"sample_{index}"
        directory.mkdir()
        physical = np.array([[0, 0, 0], [.16, 0, 0], [.16, .12, 0], [0, .12, 0]], np.float32)
        rvec = cv2.Rodrigues(camera_target[:3, :3])[0]
        projected = cv2.projectPoints(physical, rvec, camera_target[:3, 3], self.k, self.distortion)[0]
        source = np.array([[0, 0], [1599, 0], [1599, 1199], [0, 1199]], np.float32)
        homography = cv2.getPerspectiveTransform(source, projected.reshape(4, 2).astype(np.float32))
        gray = cv2.warpPerspective(self.board_image, homography, (640, 480), borderValue=255)
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(str(directory / "color.png"), color)
        cv2.imwrite(str(directory / "depth.png"), np.full((480, 640), 500, np.uint16))
        metadata = directory / "snapshot.yml"
        storage = cv2.FileStorage(str(metadata), cv2.FILE_STORAGE_WRITE)
        values = {"color": "color.png", "depth": "depth.png", "depth_unit_m": .001,
                  "aligned_to_color": 1, "width": 640, "height": 480,
                  "camera_serial": "synthetic-charuco", "host_received_ms": 1000. + index,
                  "depth_host_received_ms": 1000. + index, "color_device_us": 1000000. + index,
                  "depth_device_us": 1000000. + index, "rgbd_pairing": "device_timestamp",
                  "motion_guard": 1, "visual_motion_state": "moving" if moving else "still",
                  "motion_stable_ms": 0. if moving else 1000., "motion_tracks": 50,
                  "motion_cells": 10}
        for key, value in values.items():
            storage.write(key, value)
        storage.write("camera_matrix", self.k)
        storage.write("distortion", self.distortion)
        storage.release()
        return metadata

    def config(self):
        return {"type": "charuco", "inner_columns": None, "inner_rows": None,
                "squares_x": 8, "squares_y": 6, "square_size_m": .02,
                "marker_size_m": .0144, "dictionary": "DICT_4X4_50"}

    def test_snapshot_requires_visual_stability_and_detected_ids(self):
        pose = transform([.08, -.1, .03], [-.08, -.06, .5])
        still = self.snapshot(0, pose)
        checked = SESSION.inspect_snapshot(still, self.config())
        self.assertIsNotNone(checked)
        self.assertGreaterEqual(checked["corners"], 20)
        moving = self.snapshot(1, pose, moving=True)
        self.assertIsNone(SESSION.inspect_snapshot(moving, self.config()))

    def test_charuco_solver_recovers_known_eye_in_hand_transform(self):
        poses = [
            ([.28, -.22, .05], [-.08, -.06, .55]),
            ([-.32, .20, .08], [-.07, -.05, .58]),
            ([.35, .08, -.24], [-.09, -.04, .60]),
            ([-.18, -.36, .16], [-.06, -.07, .58]),
            ([.42, -.12, .24], [-.09, -.07, .64]),
            ([-.38, -.10, -.20], [-.06, -.04, .62]),
            ([.20, .34, -.12], [-.10, -.05, .64]),
            ([-.25, .30, .28], [-.07, -.08, .65]),
            ([.36, -.30, -.15], [-.08, -.06, .68]),
            ([-.40, .05, .25], [-.07, -.07, .66]),
        ]
        rows = []
        for index, (rotation, translation) in enumerate(poses):
            camera_target = transform(rotation, translation)
            snapshot = self.snapshot(index, camera_target)
            base_end = self.fixed_target @ np.linalg.inv(camera_target) @ np.linalg.inv(self.truth)
            rows.append((snapshot, base_end))
        samples = self.root / "samples.yml"
        storage = cv2.FileStorage(str(samples), cv2.FILE_STORAGE_WRITE)
        storage.write("translation_unit", "m")
        storage.write("board_type", "charuco")
        storage.write("board_squares_x", 8)
        storage.write("board_squares_y", 6)
        storage.write("square_size_m", .02)
        storage.write("marker_size_m", .0144)
        storage.write("aruco_dictionary", "DICT_4X4_50")
        storage.startWriteStruct("samples", cv2.FileNode_SEQ)
        for snapshot, base_end in rows:
            storage.startWriteStruct("", cv2.FileNode_MAP)
            storage.write("snapshot", str(snapshot))
            storage.write("T_base_end", base_end)
            storage.endWriteStruct()
        storage.endWriteStruct()
        storage.release()
        output = self.root / "handeye.yml"
        process = subprocess.run([str(ROOT / "build/handeye_calibrate"), str(samples), str(output)],
                                 capture_output=True, text=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stdout + process.stderr)
        result = cv2.FileStorage(str(output), cv2.FILE_STORAGE_READ)
        estimated = result.getNode("T_end_camera").mat()
        self.assertLess(np.linalg.norm(estimated[:3, 3] - self.truth[:3, 3]), .004)
        difference = estimated[:3, :3].T @ self.truth[:3, :3]
        angle = np.arccos(np.clip((np.trace(difference) - 1) / 2, -1, 1))
        self.assertLess(angle, np.deg2rad(.5))
        self.assertEqual(int(result.getNode("verified").real()), 0)
        self.assertEqual(int(result.getNode("quality_gate_pass").real()), 1)
        self.assertEqual(result.getNode("target_translation_error_m_per_sample").mat().size, 10)
        self.assertEqual(result.getNode("board_type").string(), "charuco")
        result.release()


if __name__ == "__main__":
    unittest.main()
