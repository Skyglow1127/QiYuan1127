#!/usr/bin/env python3
"""Passively verify the moved CAN source; never record episodes or send CAN."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

from lerobot_robot_piper_shared.devices import IO_CONFIG, PROOF, WORKSPACE, inspect_endpoint, proof_path
from lerobot_robot_piper_shared.transport import PassiveCAN


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    if not 2 <= args.seconds <= 300:
        parser.error('seconds must be 2..300')
    config = json.loads(IO_CONFIG.read_text())
    if config.get('transport', {}).get('kind') != 'ssh':
        parser.error('此工具只核验配置中的 KR260 SSH 接收端')
    endpoint = inspect_endpoint(config)
    if not endpoint['link']['ready']:
        parser.exit(1, 'KR260 can0 尚未处于 UP + LISTEN-ONLY + 1 Mbps。\n')
    previous = json.loads(PROOF.read_text())
    if previous['mapping'] != config['mapping'] or previous['joint_unit'] != 'deg' or previous['gripper_mode'] != 'width':
        parser.exit(1, '当前映射或单位与原有物理主从证据不符，拒绝继承。\n')
    tag = datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S')
    directory = WORKSPACE/'logs/kr260_can_verification'/tag
    directory.mkdir(parents=True, exist_ok=False)
    raw_path = directory/'can.jsonl'
    bus = PassiveCAN(config)
    snapshot = None
    try:
        with raw_path.open('w', buffering=65536) as raw:
            bus.attach_raw(raw)
            print('仅接收核验已启动。请由操作者在空位小幅调整主臂及手指机构，使执行臂产生完整新目标；本工具不发送 CAN。', flush=True)
            deadline = time.monotonic()+args.seconds
            while time.monotonic() < deadline:
                snapshot = bus.snapshot()
                fresh = all(snapshot['ages_ms'].get(f'action_{part}', 1e9) <= config['max_can_age_ms'] for part in range(4))
                width = snapshot.get('state_gripper_mode') == snapshot.get('action_gripper_mode') == 'width'
                if snapshot['ready'] and fresh and width:
                    difference = max(abs(a-b) for a, b in zip(snapshot['state'][:6], snapshot['action'][:6], strict=True))
                    if difference <= config['position_target_hold']['joint_tolerance_deg']:
                        break
                time.sleep(.005)
            else:
                (directory/'incomplete.json').write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+'\n')
                raise RuntimeError('未收到同时有效的完整新主臂目标、反馈和宽度模式；未生成核验证据。'+str(snapshot.get('error') or snapshot.get('issues')))
            bus.detach_raw()
    finally:
        bus.close()
    # Check that USB/receiver identity and mode stayed the same throughout.
    after = inspect_endpoint(config)
    if endpoint['interface_identity'] != after['interface_identity'] or not after['link']['ready']:
        raise RuntimeError('核验过程中接收端身份或监听状态改变，未更新 proof')
    evidence_path = directory/'snapshot.json'
    evidence_path.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2)+'\n')
    proof = {'at_unix': time.time(), 'interface_identity': after['interface_identity'],
        'mapping': config['mapping'], 'joint_unit': 'deg', 'gripper_mode': 'width',
        'current_source_verified': True,
        'checks': ['remote_receiver_identity', 'kernel_receive_timestamps', 'up_listen_only_1mbps',
                   'live_follower_feedback', 'complete_fresh_bus_targets', 'gripper_width_units',
                   'joint_target_feedback_consistency'],
        'evidence_file': str(evidence_path), 'raw_can_file': str(raw_path),
        'inherited_mapping_proof': str(PROOF),
        'inherited_mapping_sha256': hashlib.sha256(PROOF.read_bytes()).hexdigest(),
        'physical_leader_provenance': 'Inherited user-confirmed physical leader/follower and gripper mapping; current moved receiver and complete target/feedback frames verified passively.',
        'unit_conversion': previous.get('unit_conversion'), 'robot_commands_sent': 0}
    path = proof_path(config, IO_CONFIG)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(proof, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)
    print('KR260 当前目标/反馈核验通过；旧 AIPC 映射证据保留。证据：'+str(evidence_path), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError) as error:
        raise SystemExit(str(error))
