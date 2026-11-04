#!/usr/bin/env python3
"""Prepare arguments and exec the installed, unmodified lerobot-record command."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import sys
import time
from zoneinfo import ZoneInfo

import pyarrow.parquet as pq

from lerobot_robot_piper_shared.devices import (
    IO_CONFIG, PiperHttpCamera, default_cameras, check_endpoint,
)
from lerobot_robot_piper_shared.transport import PassiveCAN

ROOT = Path(os.environ.get('LEROBOT_WORKSPACE', '/home/aup/Lerobot'))


def default_recording_root():
    stamp = datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S')
    return ROOT/'datasets'/f'rad_place_{stamp}'


def preflight():
    config = json.loads(IO_CONFIG.read_text())
    check_endpoint(config, IO_CONFIG)
    bus = PassiveCAN(config)
    try:
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            snapshot = bus.snapshot()
            if snapshot['state'] is not None and not snapshot['observation_issues']: break
            time.sleep(.01)
        else:
            raise RuntimeError('没有实时执行臂反馈：'+str(snapshot.get('error') or snapshot.get('observation_issues')))
    finally:
        bus.close()
    for cfg in default_cameras().values():
        camera = PiperHttpCamera(cfg)
        try: camera.connect()
        finally: camera.disconnect()
    print('预检通过：CAN反馈与双相机有效，录制器只监听。正式连接仍需收到一组新主臂目标。', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=1)
    parser.add_argument('--episode-seconds', type=float, default=30)
    parser.add_argument('--reset-seconds', type=float, default=15)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    task = json.loads(IO_CONFIG.read_text())['task']
    if args.episodes < 1 or not 2 <= args.episode_seconds <= 60 or not 0 <= args.reset_seconds <= 45:
        parser.error('episodes至少1，单条时长2~60秒，复位时间0~45秒')
    if args.resume and not args.output:
        parser.error('续录需要 --output 指定已有的官方录制数据集目录')
    if args.check:
        try: preflight()
        except (OSError, RuntimeError) as error: parser.exit(1, f'预检未通过：{error}\n')
        return
    try:
        root = (args.output or default_recording_root()).expanduser().resolve()
    except (OSError, ValueError, KeyError) as error:
        parser.exit(1, f'无法生成录制目录：{error}\n')
    if root.exists() and not args.resume:
        parser.error(f'目录已存在，拒绝覆盖：{root}；续录请加 --resume')
    if args.resume:
        info = json.loads((root/'meta/info.json').read_text())
        if info['robot_type'] != 'piper_shared_observer':
            parser.error('该目录不是本适配器通过官方录制器创建的数据集')
        recorded_tasks = {row['task'] for row in pq.read_table(root/'meta/tasks.parquet').to_pylist()}
        if recorded_tasks != {task}:
            parser.error('已有数据集的任务文字与当前任务不同，请使用新目录，避免混入不同任务的示范')
    argv = ['lerobot-record', '--robot.type=piper_shared_observer', '--robot.id=piper_follower',
        f'--robot.data_root={root}', '--teleop.type=piper_shared_leader', '--teleop.id=piper_leader',
        '--dataset.repo_id=local/piper_red_cube_official', f'--dataset.root={root}',
        '--dataset.no_stamp=true', '--dataset.fps=20', f'--dataset.num_episodes={args.episodes}',
        f'--dataset.episode_time_s={args.episode_seconds}', f'--dataset.reset_time_s={args.reset_seconds}',
        f'--dataset.single_task={task}',
        '--dataset.push_to_hub=false', '--dataset.streaming_encoding=true',
        '--dataset.rgb_encoder.vcodec=h264', '--dataset.rgb_encoder.crf=18',
        '--dataset.rgb_encoder.preset=fast', '--dataset.encoder_threads=2',
        '--display_data=false', '--play_sounds=false']
    if args.resume: argv.append('--resume=true')
    print('实际调用官方入口：\n'+shlex.join(argv), flush=True)
    print('六关节数据为度deg，夹爪宽度数据为m。终端按n结束当前段，r重录，q退出。', flush=True)
    print('连接完成后自动开始；请提前放好方块。首次连接需收到一次新主臂目标。', flush=True)
    if args.dry_run:
        print('仅显示命令；未连接硬件，未创建数据集。')
        return
    try: preflight()
    except (OSError, RuntimeError) as error: parser.exit(1, f'预检未通过：{error}\n')
    (ROOT/'logs/official_record_last.json').write_text(json.dumps({
        'root':str(root),'requested_episodes':args.episodes,'command':argv,
        'recording_engine':'official lerobot-record'},ensure_ascii=False,indent=2)+'\n')
    # Use the official terminal keyboard backend; keep this terminal focused.
    env = dict(os.environ)
    env.pop('DISPLAY', None)
    env.pop('WAYLAND_DISPLAY', None)
    os.execvpe(argv[0], argv, env)


if __name__ == '__main__': main()
