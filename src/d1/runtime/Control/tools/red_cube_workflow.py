#!/usr/bin/env python3
"""红块抓放统一入口：默认只读检查、识别和规划；批准具体 manifest 后才执行。

复用现有执行器，不自动上电、使能、归零、恢复或重试。执行时在本终端
完成原执行器要求的现场视觉确认，无须助手写确认文件。
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import datetime
import hashlib
import json
import math
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools'
STATUS_URL = 'http://127.0.0.1:18763/status'
CAMERAS = {'global': (18761, 'CC1365302K0'), 'wrist': (18760, 'CC1365301EY')}
GATE_WORDS = {
    'physical_alignment_verified': ('确认可夹', '两路画面中方块在张开的指间，闭合路径有净空'),
    'physical_grasp_verified': ('确认夹住', '两路画面中方块已抬起且仍被夹持'),
    'physical_cycle_verified': ('确认已放回', '方块已回到审核位置、夹爪已松开且机械臂已退离'),
}


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def get_json(url):
    started = time.monotonic()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(url, timeout=1) as response:
        value = json.load(response)
    return value, (time.monotonic() - started) * 1000


def existing_limits():
    # Import only definitions: no CliTransport is constructed, no DDS writer.
    sys.path.insert(0, str(ROOT / 'ros2_ws/src/d1_pick_place'))
    from d1_pick_place.sdk_executor import LIMITS
    return LIMITS.tolist()


def assess_robot(status, elapsed_ms, limits):
    reasons = []
    if status.get('schema') != 'd1-joint-display-v1' or status.get('display_only') is not True:
        return ['只读反馈服务格式不匹配'], None
    age = status.get('age_ms')
    if status.get('fresh') is not True or not finite(age) or not 0 <= age + elapsed_ms <= 600:
        reasons.append('关节反馈缺失或接收时帧龄超过 0.6 秒')
    joints = status.get('joints', [])
    if len(joints) != 7 or any(j.get('id') != f'J{i}' or not finite(j.get('position'))
                               for i, j in enumerate(joints)):
        return reasons + ['需要 J0～J6 的完整有限反馈，禁止使用 last_position 代替实时反馈'], None
    angles = [j['position'] for j in joints]
    for i, (value, (low, high)) in enumerate(zip(angles, limits)):
        if not low - 1e-5 <= value <= high + 1e-5:
            reasons.append(f'J{i}={value:.3f} 超出现有限位 [{low:g}, {high:g}]')
    flags = status.get('protocol_status_flags', {})
    ages = status.get('status_age_ms', {})
    for key, wanted in [('power_status', 1), ('enable_status', 1), ('error_status', 0)]:
        field_age = ages.get(key)
        if flags.get(key) != wanted:
            reasons.append(f'控制软件标志 {key}={flags.get(key)!r}，现有执行器要求 {wanted}')
        if not finite(field_age) or not 0 <= field_age + elapsed_ms <= 600:
            reasons.append(f'{key} 标志缺失或超过 0.6 秒')
    if status.get('robot_commands_sent') != 0:
        reasons.append('反馈端点未声明零指令')
    return reasons, angles


def preflight(folder, backend, endpoint):
    started = time.monotonic()
    report = dict(schema='d1-red-cube-preflight-v1', ready=False, robot_commands_sent=0,
                  blockers=[], cameras={}, physical_hold_confirmed=None,
                  note='power/enable/error 为通信节点软件变量，不证明供电、扭矩或无硬件故障。')
    try:
        status, elapsed = get_json(STATUS_URL)
        save(folder / 'robot_status.json', status)
        blockers, angles = assess_robot(status, elapsed, existing_limits())
        report.update(blockers=blockers, sdk_angles=angles, robot_status_http_ms=elapsed)
        # Fail before images, inference or planning on known robot faults.
        if not blockers:
            urls = {role: f'http://127.0.0.1:{port}/status'
                    for role, (port, _) in CAMERAS.items()}
            if endpoint:
                urls['vision'] = endpoint.rstrip('/') + '/health'
            else:
                report['vision'] = {'checked_by': 'approved_manifest_target_recheck_in_existing_runner'}
            with ThreadPoolExecutor(max_workers=3) as pool:
                requests = {key: pool.submit(get_json, url) for key, url in urls.items()}
                for role, future in requests.items():
                    try:
                        data, elapsed = future.result()
                        save(folder / (role + '_status.json'), data)
                        if role == 'vision':
                            report['vision'] = dict(backend=backend, endpoint=endpoint, http_ms=elapsed,
                                                    status=data)
                            # Inference contract, not /health, verifies real NPU execution.
                            continue
                        color = data.get('streams', {}).get('color', {})
                        profile = data.get('profiles', {}).get('color', {})
                        age = color.get('age_ms')
                        if (data.get('device', {}).get('serial') != CAMERAS[role][1]
                                or data.get('error') or data.get('stopped') is not False
                                or not finite(age) or not 0 <= age + elapsed <= 250
                                or (profile.get('width'), profile.get('height')) != (640, 480)):
                            report['blockers'].append(role + ' 相机身份、640×480 图像或帧龄检查失败')
                        report['cameras'][role] = dict(age_ms=age, http_ms=elapsed,
                                                      serial=data.get('device', {}).get('serial'))
                    except Exception as error:
                        report['blockers'].append(f'{role} 只读服务不可用：{error}')
    except Exception as error:
        report['blockers'].append('机械臂只读预检失败：' + str(error))
    report.update(ready=not report['blockers'], elapsed_s=time.monotonic() - started)
    save(folder / 'preflight.json', report)
    return report


def stop_child(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()


def run_logged(argv, folder, timeout=1200):
    folder.mkdir(parents=True, exist_ok=False)
    save(folder / 'argv.json', list(map(str, argv)))
    with (folder / 'output.log').open('w') as stream:
        proc = subprocess.Popen(list(map(str, argv)), stdout=stream, stderr=subprocess.STDOUT)
        try:
            code = proc.wait(timeout=timeout)
        except BaseException:
            stop_child(proc)
            raise
    if code:
        raise RuntimeError(f'{Path(argv[1]).name} 退出 {code}；查看 {folder / "output.log"}')


def perception_command(args, output):
    return [sys.executable, TOOLS / 'prepare_accel_grasp.py', '--capture-live',
            '--backend', args.backend, '--preprocess', 'host', '--endpoint', args.endpoint,
            '--output', output]


def planning_command(args, perception, angles, output):
    argv = [sys.executable, TOOLS / 'run_grasp_cycle.py', '--pose', perception / 'pose.json',
            '--recognition-report', perception / 'result.json', '--expected-angles', *angles,
            '--close-target', args.close_target, '--open-target', args.open_target,
            '--yaw-deg', args.yaw_deg, '--style', 'top',
            '--grasp-height-offset-m', args.grasp_height_offset_m,
            '--lift-height-m', args.lift_height_m, '--retreat-height-m', args.retreat_height_m,
            '--timing-profile', 'collinear_bounded_v1',
            '--continuation-policy', args.continuation_policy, '--output', output]
    if args.keep_start_orientation:
        argv.append('--keep-start-orientation')
    if args.include_pregrasp:
        argv.append('--include-pregrasp')
    if args.accept_stable_close_shortfall:
        argv.append('--accept-stable-close-shortfall')
    return argv


def execution_command(args, output):
    return [sys.executable, TOOLS / 'run_grasp_cycle.py', '--execute',
            '--manifest', args.manifest.resolve(), '--approval-sha256', args.approval_sha256,
            '--approval-text', args.approval_text, '--output', output]


def make_review(manifest_path):
    manifest = read(manifest_path)
    stages = []
    for stage in manifest['stages']:
        plan = stage.get('sdk_plan', stage.get('gripper_plan'))
        stages.append(dict(name=stage['name'], duration_s=plan['duration_s'],
                           target_tcp=stage.get('target_tcp'),
                           gripper_target_sdk=stage.get('gripper_target')))
    gates = ['抬升后：确认方块仍被夹持', '放回并退离后：确认完整抓放结果']
    if manifest.get('continuation_policy') == 'task_geometry':
        gates.insert(0, '接近完成、闭合前：确认方块在指间且路径有净空')
    return dict(manifest=str(manifest_path.resolve()), approval_sha256=sha(manifest_path),
                target=manifest['target'], parameters=manifest['spec'], limits=manifest['limits'],
                stages=stages, repeats=1, place='原审核位置放回，随后退离',
                human_visual_gates=gates, human_visual_gate_count=len(gates),
                unattended=False,
                stop_conditions=['反馈陈旧/越界、相机陈旧、执行器报告故障、路径/目标偏离审批范围',
                                 '到位失败、未获视觉确认或人为中断；停止后不自动重试/恢复'],
                physical_grasp_executed=False, requires_explicit_user_approval=True)


def gate_evidence(request_path, output, approval):
    request = read(request_path)
    if request.get('approval_sha256') != approval:
        raise ValueError('视觉确认请求与批准 manifest 不匹配')
    fields = [key for key in GATE_WORDS if request.get('required', {}).get(key) is True]
    if len(fields) != 1:
        raise ValueError('未知视觉确认类型')
    observation = Path(request['observation']).resolve()
    gate = Path(request['gate_file']).resolve()
    if not observation.is_relative_to(output.resolve()) or gate.parent != output.resolve():
        raise ValueError('视觉请求路径越出本轮执行目录')
    if sha(observation) != request['observation_sha256']:
        raise ValueError('视觉观测已改变')
    images = {role: observation.parent / f'{role}.jpg' for role in CAMERAS}
    if {role: sha(path) for role, path in images.items()} != request['image_sha256']:
        raise ValueError('视觉图像已改变')
    return request, fields[0], images, gate


def terminal_execution(args, output, log_folder):
    argv = execution_command(args, output)
    log_folder.mkdir(parents=True, exist_ok=False)
    save(log_folder / 'argv.json', list(map(str, argv)))
    handled = set()
    last_state = None
    with (log_folder / 'output.log').open('w') as stream:
        proc = subprocess.Popen(list(map(str, argv)), stdout=stream, stderr=subprocess.STDOUT)
        try:
            while proc.poll() is None:
                # The original runner owns every movement and its failure policy.
                result_path = output / 'result.json'
                try:
                    state = read(result_path).get('state')
                    if state != last_state:
                        print('执行状态：' + str(state), flush=True)
                        last_state = state
                except (FileNotFoundError, json.JSONDecodeError):
                    pass
                for path in sorted(output.glob('*verification_request.json')):
                    if path in handled:
                        continue
                    try:
                        request, field, images, gate = gate_evidence(path, output, args.approval_sha256)
                    except json.JSONDecodeError:
                        continue  # Runner may still be writing the file.
                    if gate.exists():
                        raise RuntimeError('检测到非本终端创建的视觉确认，停止核查')
                    word, meaning = GATE_WORDS[field]
                    print('\n现场核对：' + meaning, flush=True)
                    for role, image in images.items():
                        print(f'  {role}: {image.as_uri()}', flush=True)
                    print(f'看清实际状态和图像后输入“{word}”；其他输入/超时均停止：', flush=True)
                    while proc.poll() is None:
                        if select.select([sys.stdin], [], [], .2)[0]:
                            answer = sys.stdin.readline().strip()
                            if answer != word:
                                raise RuntimeError('现场未确认，停止后续动作')
                            # Recheck immutable observation/images after the human wait.
                            checked, checked_field, _, checked_gate = gate_evidence(path, output, args.approval_sha256)
                            if checked != request or checked_field != field or checked_gate != gate:
                                raise ValueError('等待期间视觉确认请求已改变')
                            payload = dict(approval_sha256=args.approval_sha256,
                                           observation_sha256=request['observation_sha256'],
                                           image_sha256=request['image_sha256'], verifier='human',
                                           **{field: True})
                            payload.update(confirmation_text=answer, confirmed_at_unix=time.time())
                            # Atomic replacement: the runner cannot read partial JSON.
                            temporary = gate.with_name(gate.name + '.terminal.tmp')
                            with temporary.open('x') as target:
                                json.dump(payload, target, ensure_ascii=False, allow_nan=False)
                            temporary.replace(gate)
                            handled.add(path)
                            break
                    if proc.poll() is not None:
                        break
                time.sleep(.2)
            code = proc.wait()
        except BaseException:
            stop_child(proc)
            raise
    if code:
        raise RuntimeError(f'执行器停止 ({code})；查看 {output / "result.json"} 和 {log_folder / "output.log"}')
    result = read(output / 'result.json')
    if result.get('completed') is not True:
        raise RuntimeError('执行器未报告整轮完成')
    return result


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog='最短用法：./run_red_cube.sh（准备计划）；./run_red_cube.sh --preflight-only（只检查）；'
               './run_red_cube.sh --execute --manifest <manifest.json> --approval-sha256 <SHA256> '
               '--approval-text "实际批准原文"（批准后执行一次，保留终端视觉确认）。')
    p.add_argument('--preflight-only', '--dry-run', action='store_true',
                   help='仅 GET 现有只读服务并保存检查；不识别、不规划、不运动')
    p.add_argument('--execute', action='store_true', help='执行已明确批准的具体 manifest；本终端保留原有视觉确认')
    p.add_argument('--manifest', type=Path)
    p.add_argument('--approval-sha256')
    p.add_argument('--approval-text', help='用户对这份目标、路径、参数的实际批准原文；哈希本身不表示批准')
    p.add_argument('--output', type=Path, help='全新结果目录，省略则在 logs 下创建带时间戳目录')
    p.add_argument('--backend', choices=['npu', 'cpu', 'hsv'], default='npu')
    p.add_argument('--endpoint', help='现有视觉服务地址；不会启动或重启服务')
    p.add_argument('--cube-size-mm', type=float, choices=[40.], default=40., help='本流程仅支持已有 40 mm 红块')
    p.add_argument('--lift-height-m', type=float, default=.10, help='沿桌面法线抬升，最大为既有限制 0.10 m')
    p.add_argument('--retreat-height-m', type=float, default=.08)
    p.add_argument('--close-target', type=float, default=29.6, help='历史候选 J6 SDK 单位，不是力或开口毫米')
    p.add_argument('--open-target', type=float, default=36., help='初始反馈必须已在此位置附近；不会自动张开')
    p.add_argument('--yaw-deg', type=float, default=90.)
    p.add_argument('--grasp-height-offset-m', type=float, default=.01)
    p.add_argument('--keep-start-orientation', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--include-pregrasp', action=argparse.BooleanOptionalAction, default=True)
    p.add_argument('--continuation-policy', choices=['reviewed_path', 'task_geometry'], default='reviewed_path')
    p.add_argument('--accept-stable-close-shortfall', action='store_true',
                   help='把现有受限闭合未到位判据纳入待审批计划；不能等同力/接触检测')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if args.execute:
        if args.preflight_only or args.manifest is None or not args.approval_sha256 or not (args.approval_text or '').strip():
            p.error('--execute 必须同时提供 --manifest、--approval-sha256 和实际 --approval-text，不能同时 --dry-run')
        if not sys.stdin.isatty():
            p.error('执行需要交互终端作现场视觉确认；请在本机终端运行，禁止自动确认')
    elif any((args.manifest, args.approval_sha256, args.approval_text)):
        p.error('manifest/approval 参数仅用于 --execute；默认重新采集并准备计划')
    if not finite(args.lift_height_m) or not 0 < args.lift_height_m <= .10:
        p.error('抬升高度必须位于 (0, 0.10] m，不改变既有限制')
    args.endpoint = args.endpoint or {'npu': 'http://127.0.0.1:18820',
                                     'cpu': 'http://127.0.0.1:18821',
                                     'hsv': 'http://127.0.0.1:18822'}[args.backend]
    if args.output is None:
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        args.output = ROOT / 'logs' / ('red_cube_' + stamp)
    args.output = args.output.resolve()
    report = dict(schema='d1-red-cube-workflow-v1', state='starting', output=str(args.output),
                  mode='execute' if args.execute else 'preflight' if args.preflight_only else 'prepare',
                  hardware_commands_may_have_been_sent=False)
    created = False
    try:
        args.output.mkdir(parents=True, exist_ok=False)
        created = True
        print('只读快速预检；结果目录：' + str(args.output), flush=True)
        # Execute uses the backend bound in the approved manifest, not current
        # prepare defaults. The existing runner validates/rechecks that contract.
        pre = preflight(args.output, args.backend, None if args.execute else args.endpoint)
        if not pre['ready']:
            report.update(state='blocked', blockers=pre['blockers'], robot_commands_sent=0)
            for reason in pre['blockers']:
                print('阻止：' + reason, flush=True)
            print('未启动识别/规划/执行，未发出机械臂指令。', flush=True)
            save(args.output / 'workflow.json', report)
            return 2
        if args.preflight_only:
            report.update(state='preflight_passed', robot_commands_sent=0)
        elif args.execute:
            print('核验已批准 manifest；全部参数以 manifest 为准。', flush=True)
            run_logged([sys.executable, TOOLS / 'run_grasp_cycle.py', '--manifest', args.manifest,
                        '--approval-sha256', args.approval_sha256], args.output / 'manifest_check', 30)
            report['hardware_commands_may_have_been_sent'] = True
            result = terminal_execution(args, args.output / 'execution', args.output / 'executor_call')
            report.update(state='completed', execution_result=str(args.output / 'execution/result.json'),
                          physical_cycle_verified=result.get('physical_cycle_verified'))
        else:
            if abs(pre['sdk_angles'][6] - args.open_target) > .5:
                raise ValueError(f'J6={pre["sdk_angles"][6]:.3f} 不在待审核 open-target={args.open_target:g} 附近；不会自动打开')
            perception = args.output / 'perception'
            print('采集当前画面并识别 40 mm 红块……', flush=True)
            run_logged(perception_command(args, perception), args.output / 'perception_call', 60)
            result = read(perception / 'result.json')
            if not result.get('completed') or not result.get('execution_candidate_eligible'):
                raise RuntimeError('实时识别未形成可用候选；查看 perception/result.json')
            print('调用现有抓放规划器，生成完整路径和审批 manifest……', flush=True)
            plan = args.output / 'plan'
            observed_angles = read(perception / 'pose.json')['sdk_start']
            run_logged(planning_command(args, perception, observed_angles, plan), args.output / 'planning_call')
            review = make_review(plan / 'manifest.json')
            save(args.output / 'review.json', review)
            report.update(state='awaiting_plan_approval', robot_commands_sent=0,
                          review=str(args.output / 'review.json'), **{k: review[k] for k in ('manifest', 'approval_sha256')})
            print('计划已准备，尚未执行。目标、路径、参数：' + str(args.output / 'review.json'), flush=True)
            print('manifest：' + review['manifest'] + '\nSHA-256：' + review['approval_sha256'], flush=True)
            print('取得对具体方案的明确批准后，使用本入口 --execute --manifest <上述文件> '
                  '--approval-sha256 <上述哈希> --approval-text <实际批准原文>。', flush=True)
        save(args.output / 'workflow.json', report)
        print('结果：' + report['state'], flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as error:
        report.update(state='stopped_no_recovery', error=str(error) or type(error).__name__)
        if created and not (args.output / 'workflow.json').exists():
            save(args.output / 'workflow.json', report)
        print('停止：' + report['error'], file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
