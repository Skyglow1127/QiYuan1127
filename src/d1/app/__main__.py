import argparse
import json
from pathlib import Path
import sys

from .backend import Backend, DEFAULTS, check_environment

def main():
    parser=argparse.ArgumentParser(description='D1 OpenCV 单次抓放程序；默认打开图形界面并只准备路径')
    parser.add_argument('--demo',action='store_true',help='模拟界面，不连接设备')
    parser.add_argument('--demo-run',action='store_true',help='无界面演示全流程，仅模拟数据，零硬件指令')
    parser.add_argument('--check',action='store_true',help='仅核验本地文件和运行依赖，不连接设备')
    parser.add_argument('--no-auto-prepare',action='store_true')
    args=parser.parse_args()
    if args.check:
        issues=check_environment()
        print(json.dumps({'ready':not issues,'issues':issues,'robot_commands_sent':0},ensure_ascii=False,indent=2))
        return int(bool(issues))
    if args.demo_run:
        backend=Backend(mode='demo',emit=lambda e:print(json.dumps(e,ensure_ascii=False),flush=True))
        review=backend.prepare(DEFAULTS)
        if not review.get('ready'): return 1
        result=backend.execute(review,'纯软件演示；不授权任何实机动作')
        return 0 if result['status']=='demo_completed' else 1
    from .gui import main as gui_main
    options=[]
    if args.demo:options.append('--demo')
    if args.no_auto_prepare:options.append('--no-auto-prepare')
    return gui_main(options)

if __name__=='__main__':
    raise SystemExit(main())
