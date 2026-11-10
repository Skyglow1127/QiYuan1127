#!/usr/bin/env python3
"""Collect and export eye-in-hand samples from the AIPC-connected Orbbec camera.

Robot access is read-only. Capture stores raw joint feedback and an immutable
camera snapshot. Export is deliberately gated by measured board evidence and a
validated SDK-to-URDF joint mapping.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT.parent / "D1" / "Control"
CONTROL_TOOL = CONTROL / "tools" / "kr260_handeye.py"


def load_control_tool():
    spec = importlib.util.spec_from_file_location("d1_handeye_evidence", CONTROL_TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    temporary = Path(str(path) + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def running_detectors():
    result = subprocess.run(["pgrep", "-af", str(ROOT / "build" / "detect")],
                            capture_output=True, text=True)
    return [line for line in result.stdout.splitlines() if line.strip()]


def read_board(path, require_validated=False):
    board = json.loads(path.read_text())
    board_type = board.get("type", "chessboard")
    if board_type not in ("chessboard", "charuco"):
        raise ValueError("标定板 type 只支持 chessboard 或 charuco")
    if board_type == "charuco":
        cols, rows = board.get("squares_x"), board.get("squares_y")
    else:
        cols, rows = board.get("inner_columns"), board.get("inner_rows")
    square = board.get("square_size_m")
    if type(cols) is not int or type(rows) is not int or not 3 <= cols <= 30 or not 3 <= rows <= 30:
        raise ValueError("棋盘内角点行列必须为 3..30 的整数")
    if require_validated:
        helper = load_control_tool()
        helper.verified_evidence(board, path.parent)
        if (type(square) not in (int, float) or not math.isfinite(square)
                or not .001 < square < .1):
            raise ValueError("需要用卡尺测量并填写 square_size_m")
        if board_type == "charuco":
            marker = board.get("marker_size_m")
            if (type(marker) not in (int, float) or not math.isfinite(marker)
                    or not 0 < marker < square):
                raise ValueError("需要测量 marker_size_m，且它必须小于方格边长")
            name = board.get("dictionary", "")
            if not name.startswith("DICT_") or not hasattr(cv2.aruco, name):
                raise ValueError("不支持的 ArUco dictionary")
        if board.get("fixed_board_and_rigid_camera_confirmed") is not True:
            raise ValueError("需要确认棋盘固定、相机与 Link6 刚性固定")
        if not isinstance(board.get("camera_serial"), str) or not board["camera_serial"]:
            raise ValueError("需要填写本次标定相机的序列号")
    return board


def node_number(storage, key, default=0.0):
    node = storage.getNode(key)
    return default if node.empty() else node.real()


def inspect_snapshot(metadata, board):
    storage = cv2.FileStorage(str(metadata), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise ValueError("无法读取相机快照元数据")
    color_name = storage.getNode("color").string()
    camera_serial = storage.getNode("camera_serial").string()
    matrix = storage.getNode("camera_matrix").mat()
    distortion = storage.getNode("distortion").mat()
    guarded = int(node_number(storage, "motion_guard", 0))
    motion = storage.getNode("visual_motion_state").string() or "not_checked"
    stable_ms = node_number(storage, "motion_stable_ms", 0)
    storage.release()
    image = cv2.imread(str(metadata.parent / color_name))
    if image is None or matrix is None or matrix.shape != (3, 3):
        raise ValueError("快照图像或相机内参无效")
    if not guarded or motion != "still" or stable_ms < 600:
        return None
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if board.get("type", "chessboard") == "charuco":
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board["dictionary"]))
        charuco = cv2.aruco.CharucoBoard_create(
            board["squares_x"], board["squares_y"],
            float(board.get("square_size_m") or 1.0),
            float(board.get("marker_size_m") or .72), dictionary)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        if marker_ids is None or len(marker_ids) < 4:
            return None
        count, corners, corner_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, charuco, cameraMatrix=matrix,
            distCoeffs=distortion)
        total = (board["squares_x"] - 1) * (board["squares_y"] - 1)
        if count is None or count < max(12, math.ceil(.55 * total)):
            return None
        detected = int(count)
        identifier_span = [int(corner_ids.min()), int(corner_ids.max())]
    else:
        pattern = (board["inner_columns"], board["inner_rows"])
        found, corners = cv2.findChessboardCornersSB(
            image, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        if not found:
            return None
        detected = int(len(corners))
        identifier_span = None
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    hull = cv2.convexHull(corners).reshape(-1, 2)
    coverage = float(cv2.contourArea(hull) / (image.shape[0] * image.shape[1]))
    margin = float(min(corners[:, 0, 0].min(), corners[:, 0, 1].min(),
                       image.shape[1] - 1 - corners[:, 0, 0].max(),
                       image.shape[0] - 1 - corners[:, 0, 1].max()))
    quality = coverage * min(sharpness, 1000.0)
    return {"metadata": metadata, "camera_serial": camera_serial, "corners": detected,
            "corner_id_span": identifier_span, "coverage": coverage,
            "sharpness": sharpness, "edge_margin_px": margin, "quality": quality}


def capture(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"complete": False, "sample_ready": False,
              "robot_motion_commands_sent": False, "handeye_calibrated": False}
    helper = load_control_tool()
    observer = process = None
    try:
        board = read_board(args.board.resolve(), require_validated=False)
        active = running_detectors()
        if active:
            raise RuntimeError("相机正被实时视觉占用；请在视觉窗口按 q 退出后再采样")
        helper.check_link(args.nic)
        observer = helper.Observer(CONTROL / "build" / "arm_control", args.nic, output)
        observer.pump(2.0)
        begin = time.monotonic()
        observer.pump(helper.PRE_SECONDS + .2)
        helper.assess_window(observer.events, begin, time.monotonic())
        camera = output / "camera"
        command = [str(ROOT / "build" / "detect"), "--config", str(args.config.resolve()),
                   "--mode", "fusion", "--seconds", str(args.seconds), "--headless",
                   "--record-interval-ms", "250", "--output", str(camera)]
        with (output / "camera.log").open("w") as log:
            request_begin = time.monotonic()
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            while process.poll() is None:
                observer.pump(.1)
                helper.assess_window(observer.events, begin, time.monotonic())
            request_end = time.monotonic()
        if process.returncode:
            raise RuntimeError("相机采集失败；可能仍被其他视觉程序占用，详见 camera.log")
        observer.pump(helper.POST_SECONDS)
        end = time.monotonic()
        observation = helper.assess_window(observer.events, begin, end)
        candidates = []
        for metadata in sorted((camera / "samples").glob("frame_*.yml")):
            checked = inspect_snapshot(metadata, board)
            if checked is not None:
                candidates.append(checked)
        if not candidates:
            raise ValueError("采集期间没有同时满足停稳和完整棋盘角点的图像")
        selected = max(candidates, key=lambda item: item["quality"])
        expected_serial = board.get("camera_serial")
        if expected_serial and selected["camera_serial"] != expected_serial:
            raise ValueError("相机序列号与标定板配置记录不一致")
        source = selected["metadata"]
        frozen = output / "snapshot"
        frozen.mkdir()
        storage = cv2.FileStorage(str(source), cv2.FILE_STORAGE_READ)
        names = [storage.getNode("color").string(), storage.getNode("depth").string()]
        storage.release()
        for name in names:
            shutil.copy2(source.parent / name, frozen / name)
        shutil.copy2(source, frozen / "snapshot.yml")
        report.update(complete=True, sample_ready=True, observation=observation,
                      capture_window={"begin_aipc_monotonic_s": request_begin,
                                      "end_aipc_monotonic_s": request_end},
                      board_detection={k: v for k, v in selected.items() if k != "metadata"},
                      joint_median_sdk_deg=observation["joint_median"],
                      selected_snapshot="snapshot/snapshot.yml")
        evidence = [output / "feedback.jsonl", output / "camera.log", frozen / "snapshot.yml",
                    *(frozen / name for name in names)]
        report["sha256"] = {str(path.relative_to(output)): digest(path) for path in evidence}
    except Exception as error:
        report["error"] = str(error)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if observer is not None:
            observer.close()
        save_json(output / "capture.json", report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["complete"] else 1


def export_samples(args):
    helper = load_control_tool()
    board_path = args.board.resolve()
    mapping_path = args.mapping.resolve()
    board = read_board(board_path, require_validated=True)
    mapping = json.loads(mapping_path.read_text())
    helper.verified_evidence(mapping, mapping_path.parent)
    output = args.output.resolve()
    if output.exists():
        raise ValueError("输出文件已存在，拒绝覆盖")
    rows = []
    if len({session.resolve() for session in args.sessions}) != len(args.sessions):
        raise ValueError("样本目录不可重复")
    for session in args.sessions:
        session = session.resolve()
        record = json.loads((session / "capture.json").read_text())
        if record.get("complete") is not True or record.get("sample_ready") is not True:
            raise ValueError(f"无效或未完成的采样：{session}")
        for relative, expected in record.get("sha256", {}).items():
            if digest(session / relative) != expected:
                raise ValueError(f"采样证据被修改：{session / relative}")
        pose = helper.mapped_pose(mapping_path, record["joint_median_sdk_deg"])
        snapshot = session / record["selected_snapshot"]
        checked = inspect_snapshot(snapshot, board)
        if checked is None:
            raise ValueError(f"棋盘或停稳证据复核失败：{session}")
        if checked["camera_serial"] != board.get("camera_serial"):
            raise ValueError(f"相机序列号不匹配：{session}")
        rows.append((snapshot, pose))
    if len(rows) < 8:
        raise ValueError("至少需要 8 组，建议采集 15–25 组多轴旋转姿态")
    output.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(output), cv2.FILE_STORAGE_WRITE)
    storage.write("translation_unit", "m")
    board_type = board.get("type", "chessboard")
    storage.write("board_type", board_type)
    if board_type == "charuco":
        storage.write("board_squares_x", board["squares_x"])
        storage.write("board_squares_y", board["squares_y"])
        storage.write("marker_size_m", float(board["marker_size_m"]))
        storage.write("aruco_dictionary", board["dictionary"])
    else:
        storage.write("board_columns", board["inner_columns"])
        storage.write("board_rows", board["inner_rows"])
    storage.write("square_size_m", float(board["square_size_m"]))
    storage.write("base_frame", "base_link")
    storage.write("end_frame", "Link6")
    storage.write("mapping_sha256", digest(mapping_path))
    storage.write("board_config_sha256", digest(board_path))
    storage.startWriteStruct("samples", cv2.FileNode_SEQ)
    for snapshot, pose in rows:
        storage.startWriteStruct("", cv2.FileNode_MAP)
        storage.write("snapshot", str(snapshot))
        storage.write("T_base_end", np.asarray(pose, dtype=np.float64))
        storage.endWriteStruct()
    storage.endWriteStruct()
    storage.release()
    print(f"已导出 {len(rows)} 组配对样本：{output}")
    return 0


def doctor(args):
    helper = load_control_tool()
    report = {"robot_motion_commands_sent": False, "camera_processes": running_detectors(),
              "d1_ready": False, "board_config_ready": False,
              "joint_mapping_ready": False}
    try:
        report["d1_addresses"] = helper.check_link(args.nic)
        status = subprocess.run([str(CONTROL / "build" / "arm_control"), "--nic", args.nic,
                                 "--verbose", "watch", "3"], capture_output=True, text=True, timeout=8)
        events = [event for line in status.stdout.splitlines()
                  if (event := helper.parse_feedback(line, 0)) is not None]
        angles = [event for event in events if event["kind"] == "angles"]
        states = [event for event in events if event["kind"] == "status"]
        report["angle_frames"] = len(angles)
        report["status_frames"] = len(states)
        report["latest_power_enable_error"] = states[-1]["state"] if states else None
        report["d1_ready"] = len(angles) >= 3 and len(states) >= 3 and states[-1]["state"] == [1, 1, 0]
    except Exception as error:
        report["d1_blocker"] = str(error)
    if args.board:
        try:
            read_board(args.board.resolve(), require_validated=True)
            report["board_config_ready"] = True
        except Exception as error:
            report["board_blocker"] = str(error)
    if args.mapping:
        try:
            mapping = json.loads(args.mapping.read_text())
            helper.verified_evidence(mapping, args.mapping.parent)
            helper.mapped_pose(args.mapping.resolve(), [0.] * 7)
            report["joint_mapping_ready"] = True
        except Exception as error:
            report["joint_mapping_blocker"] = str(error)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        save_json(args.output, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if all(report[key] for key in ("d1_ready", "board_config_ready", "joint_mapping_ready")) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nic", default="enp99s0")
    sub = parser.add_subparsers(dest="mode", required=True)
    capture_parser = sub.add_parser("capture")
    capture_parser.add_argument("--board", required=True, type=Path)
    capture_parser.add_argument("--config", type=Path, default=ROOT / "configs/aipc_usb_motion.yml")
    capture_parser.add_argument("--seconds", type=float, default=5)
    capture_parser.add_argument("--output", required=True, type=Path)
    export_parser = sub.add_parser("export")
    export_parser.add_argument("--board", required=True, type=Path)
    export_parser.add_argument("--mapping", required=True, type=Path)
    export_parser.add_argument("--output", required=True, type=Path)
    export_parser.add_argument("sessions", nargs="+", type=Path)
    doctor_parser = sub.add_parser("doctor")
    doctor_parser.add_argument("--board", type=Path)
    doctor_parser.add_argument("--mapping", type=Path)
    doctor_parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        if args.mode == "capture" and not 3 <= args.seconds <= 30:
            raise ValueError("采集时间必须为 3..30 秒")
        return {"capture": capture, "export": export_samples, "doctor": doctor}[args.mode](args)
    except (ValueError, RuntimeError, OSError, KeyError, subprocess.TimeoutExpired) as error:
        print(f"未完成：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
