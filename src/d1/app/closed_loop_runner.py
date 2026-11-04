"""Prepare a reviewed shared-progress READY experiment and run it once.

Use ``python -m app.closed_loop_runner prepare --method method02_shared_progress``.
Preparation observes joints and plans; execution requires the saved review.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import signal
import sys
import threading

from . import backend as backend_module
from .backend import Backend, DEFAULTS, ZONE, digest
from .execution_scope import FIXED_PREGRASP
from .reporting import load, write_json
for _directory in (backend_module.TOOLS,backend_module.CONTROL/'ros2_ws/src/d1_pick_place'):
    if str(_directory) not in sys.path: sys.path.insert(0,str(_directory))
from d1_pick_place.sdk_executor import (ARM_POSITION_TOLERANCE_DEG,
    FEEDBACK_FRESHNESS_S, FEEDBACK_RECOVERY_TIMEOUT_S, FIXED_PROGRESS_MARGIN_DEG)
from d1_pick_place.shared_progress import (MAX_PHASE_RATE, MAX_PHASE_ACCELERATION,
    SLOW_LAG_DEG, STOP_LAG_DEG, STALL_TIMEOUT_S)

METHOD = 'method02_shared_progress'
SCHEDULING_POLICY = 'shared_progress_v1'
STARTUP_ACTIONS = ['start_cameras_if_needed','power_on_if_needed']
FIXED_MOTION = dict(speed_deg_s=5., acceleration_deg_s2=10., rate_hz=10., gripper_speed_sdk_s=10.)
SETTINGS = dict(DEFAULTS, open_target=36.)


def check_live_interface():
    """Check only the configured local adapter; never reroute robot traffic."""
    contract=load(backend_module.APP/'execution_contract.json')
    nic=contract['host']['network_interface']
    if nic!='enp99s0':
        raise ValueError('当前观察与执行工具固定使用enp99s0；更换网口须同时更新适配器，不能只改合同')
    state=Path('/sys/class/net')/nic/'operstate'
    if not state.is_file() or state.read_text().strip()!='up':
        raise RuntimeError(f'D1通信网口 {nic} 未连接或未启用；无法读取当前姿态，尚未启动SDK或发送运动指令')


def _method_digest(payload):
    data={key:value for key,value in payload.items() if key!='method_approval_sha256'}
    return hashlib.sha256(json.dumps(data,sort_keys=True,ensure_ascii=False,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def _method_payload(review):
    """Describe only the exact saved backend review; no new motion plan is created."""
    folder=Path(review['run_dir'])
    return dict(schema='d1-method-review-v1',method_id=METHOD,mode=review['mode'],
        backend_review=str(folder/'review.json'),backend_review_sha256=digest(folder/'review.json'),
        run_dir=str(folder),manifest=review['manifest'],manifest_sha256=review['approval_sha256'],
        expires_at=review['expires_at'],execution_scope=FIXED_PREGRASP,
        target=review['target'],observed_joint_angles=review['observed_joint_angles'],
        parameters=review['parameters'],stages=review['stages'],
        scheduling_policy=SCHEDULING_POLICY,startup_actions=review.get('startup_actions',[]),
        execution_contract=load(backend_module.APP/'execution_contract.json'),
        execution_contract_sha256=digest(backend_module.APP/'execution_contract.json'),
        progress_controller=dict(max_phase_rate=MAX_PHASE_RATE,
            max_phase_acceleration_s_inv=MAX_PHASE_ACCELERATION,
            slowdown_lag_deg=SLOW_LAG_DEG,stop_lag_deg=STOP_LAG_DEG,
            stall_timeout_s=STALL_TIMEOUT_S,
            effective_reference_peak_speed_upper_bound_deg_s=review['parameters']['speed_deg_s']*MAX_PHASE_RATE,
            derivative_scope='modeled uninterrupted active-time reference; physical motion and wall-time holds are not certified'),
        run_scope=dict(max_executions=1,requested_stages=['prepare_gripper',FIXED_PREGRASP],
            return_to_start=False,automatic_replan=False),
        stop_conditions=dict(tracking_error_deg=3.,candidate_error_deg=FIXED_PROGRESS_MARGIN_DEG,
            phase_stall_timeout_s=STALL_TIMEOUT_S,
            total_progress_budget='4 * source_duration_s + 30 seconds',
            required_joint_feedback_timeout_s=FEEDBACK_RECOVERY_TIMEOUT_S,
            command_receipt_timeout_s=.6,held_axis_error_deg=.5,j6_tracking_error_sdk=3.,
            feedback_freshness_s=FEEDBACK_FRESHNESS_S,feedback_recovery_timeout_s=FEEDBACK_RECOVERY_TIMEOUT_S,
            endpoint_tolerance_deg=ARM_POSITION_TOLERANCE_DEG,
            endpoint_policy=review.get('endpoint_policy'),device_fault='stop future commands',
            cancellation='stop future commands; physical stop is not certified'),
        evaluation_scope='验证共享轨迹进度、整组准入与有界反馈降速；5°/s、10°/s²为参考约束，实际运动及底层覆盖语义待验证。',
        synthetic=review['mode']=='demo',physical_motion_validated=False)


def _check_method_review(review):
    if (review.get('operation')!=FIXED_PREGRASP or review.get('scheduling_policy')!=SCHEDULING_POLICY
            or review.get('startup_actions')!=STARTUP_ACTIONS):
        raise ValueError('计划不是轨迹闭环方法的固定姿态与批准启动流程')
    parameters=review.get('parameters',{})
    if any(parameters.get(key)!=value for key,value in FIXED_MOTION.items()) or parameters.get('open_target')!=36.:
        raise ValueError('轨迹闭环方法参数与固定的 5°/s、10°/s²、10 Hz、张爪 36 不匹配')
    if [stage.get('name') for stage in review.get('stages',[])]!=['prepare_gripper',FIXED_PREGRASP]:
        raise ValueError('轨迹闭环方法只能执行夹爪调整及固定预抓取移动一次')


def _summary_text(payload):
    lines=[f"方法：{payload['method_id']}",
        '演示模式：所有位置为合成数据，不证明实机效果。' if payload['synthetic'] else '实机方法：准备阶段只读，等待明确批准后执行一次。',
        payload['evaluation_scope'],
        '一次批准范围：先启动或复用双相机服务并核验两路就绪；若需要则发送一次协议上电请求并回读 power_status=1；再调整夹爪至 36 SDK、沿下列已检查路径进入固定 ready 姿态并停止。',
        '准备阶段仅读取机械臂反馈并规划固定姿态，不打开相机或恢复相机服务。',
        '上电回读是协议软件标志；执行前仍核对新鲜反馈及原批准起点，位置变化则拒绝，不自动重新规划。',
        '下发节奏：整臂共享轨迹相位，按反馈裕量协调降速；ACK计入实际周期，超期不追赶；仍逐轴提交，非硬件同步。',
        '初始实测七轴 SDK：'+json.dumps(payload['observed_joint_angles'],ensure_ascii=False),
        '目标：'+json.dumps(payload['target'],ensure_ascii=False),
        '关键参数：'+json.dumps(payload['parameters'],ensure_ascii=False),
        '共同进度参数：'+json.dumps(payload['progress_controller'],ensure_ascii=False),
        f"源曲线速度上限 {payload['parameters']['speed_deg_s']:g}°/s；预留相位变速余量后，参考峰值速度上界 {payload['progress_controller']['effective_reference_peak_speed_upper_bound_deg_s']:g}°/s，实机能力尚待验证。",
        '停止条件：'+json.dumps(payload['stop_conditions'],ensure_ascii=False),
        '所有机械臂 SDK 路径点（J0–J5，度）：']
    for stage in payload['stages']:
        if 'sdk_plan' not in stage: continue
        lines.append(f"阶段 {stage['name']}；计划时长 {stage['duration_s']:g} 秒")
        for index,waypoint in enumerate(stage['sdk_plan']['sdk']):
            lines.append(f"  {index}: "+json.dumps(waypoint,ensure_ascii=False))
    lines.extend(['不自动返回、不执行下探或抓放；故障/取消后停止后续指令，软件不证明物理停稳。',
        '有效期截至（UTC+8）：'+datetime.fromtimestamp(payload['expires_at'],ZONE).isoformat(),
        'manifest SHA-256：'+payload['manifest_sha256'],
        '方法批准 SHA-256：'+payload['method_approval_sha256'],
        '完整方法 review：'+str(Path(payload['run_dir'])/'method_review.json'),
        '本次只准备，尚未收到或填写任何批准，未执行运动。'])
    return '\n'.join(lines)+'\n'


def prepare_method(method=METHOD,*,mode='live',valid_for_s=7200.,emit=None):
    if method!=METHOD: raise ValueError('未知测试方法：'+str(method))
    if mode=='live':check_live_interface()
    instance=Backend(mode=mode,emit=emit)
    review=instance.prepare(SETTINGS,fixed_pregrasp=True,fixed_motion=FIXED_MOTION,
        scheduling_policy=SCHEDULING_POLICY,valid_for_s=valid_for_s)
    if review.get('ready') is not True:
        raise RuntimeError('方法准备未通过：'+'；'.join(review.get('blockers',[])))
    _check_method_review(review)
    payload=_method_payload(review);payload['method_approval_sha256']=_method_digest(payload)
    folder=Path(review['run_dir'])
    write_json(folder/'method_review.json',payload)
    (folder/'summary.txt').write_text(_summary_text(payload),encoding='utf-8')
    return payload


def _load_method(path,mode):
    path=Path(path).expanduser().resolve(strict=True)
    if path.name!='method_review.json' or path.parent.parent!=(backend_module.APP/'logs').resolve():
        raise ValueError('只能执行本程序 logs 目录内的 method_review.json')
    payload=load(path)
    if (not isinstance(payload,dict) or payload.get('schema')!='d1-method-review-v1'
            or payload.get('method_id')!=METHOD or payload.get('mode')!=mode
            or payload.get('method_approval_sha256')!=_method_digest(payload)):
        raise ValueError('方法 review、模式或方法批准哈希不匹配')
    review_path=path.parent/'review.json'
    if payload.get('backend_review')!=str(review_path) or payload.get('backend_review_sha256')!=digest(review_path):
        raise ValueError('方法 review 未绑定原后端 review')
    review=load(review_path)
    _check_method_review(review)
    expected=_method_payload(review);expected['method_approval_sha256']=_method_digest(expected)
    if expected!=payload: raise ValueError('方法摘要与原批准路径、参数不一致')
    return path,payload,review_path


def _method_summary(payload,result,startup,error=None,*,cameras=None):
    folder=Path(payload['run_dir']);reports=[]
    for path in sorted((folder/'execution').glob('[0-9][0-9]_*/executed/*.json')):
        if path.name not in ('result.json','summary.json'): continue
        raw=load(path,{})
        reports.append(dict(path=str(path),commands=raw.get('commands'),receipts=raw.get('receipts'),
            raw_transport=str(path.parent/'transport.jsonl'),
            approved_execution_plan=str(path.parent/'plan.json'),frames=str(path.parent/'frames'),
            scheduling_policy=raw.get('scheduling_policy'),
            scheduling_overrun_s=raw.get('scheduling_overrun_s'),
            command_group_schedule=raw.get('command_group_schedule',[]),
            shared_progress_metrics=raw.get('shared_progress_metrics'),
            phase_events=raw.get('phase_events',[]),
            wait={key:raw.get(key) for key in ('command_wait_pause_s','feedback_pacing_wait_s',
                'feedback_gap_wait_s','endpoint_seek_elapsed_s','scheduling_overrun_s')},
            error=raw.get('error'),completed=raw.get('completed')))
    details=result.get('details') or load(folder/'execution/result.json',{})
    actual=result.get('final_angles') or details.get('final_angles')
    if actual is None:
        stage=load(folder/'execution/01_fixed_pregrasp/executed/result.json',{})
        actual=stage.get('final_angles')
    target=payload['stages'][-1]['sdk_plan']['sdk'][-1]
    residual=[float(a)-float(b) for a,b in zip(actual[:6],target)] if isinstance(actual,list) and len(actual)>=6 else None
    counts=lambda key: sum(item[key] for item in reports if isinstance(item[key],int))
    return dict(schema='d1-method-summary-v1',method_id=METHOD,mode=payload['mode'],
        synthetic=payload['synthetic'],completed=result.get('pregrasp_reached') is True,
        physical_motion_validated=payload['mode']=='live' and result.get('pregrasp_reached') is True,
        evaluation_scope=payload['evaluation_scope'],status=result.get('status','failed'),
        method_approval_sha256=payload['method_approval_sha256'],manifest_sha256=payload['manifest_sha256'],
        camera_startup_result=cameras,startup_result=startup,
        startup_commands=(startup or {}).get('robot_commands_sent'),
        approved_arm_target_sdk=target,final_angles=actual,
        endpoint_error_deg=residual,max_endpoint_error_deg=max(map(abs,residual)) if residual else None,
        commands=counts('commands'),receipts=counts('receipts'),stage_reports=reports,
        timing=result.get('details',{}).get('timing'),total_time_s=result.get('total_time_s'),
        preparation_time_s=result.get('preparation_time_s'),
        stop_reason=error or result.get('error') or ('固定预抓取到位' if result.get('pregrasp_reached') else '未确认到位'),
        physical_stop_verified=result.get('physical_stop_verified',False),
        paths=dict(run_dir=str(folder),method_review=str(folder/'method_review.json'),
            backend_review=str(folder/'review.json'),user_approval=str(folder/'user_approval.json'),
            method_approval=str(folder/'method_approval.json'),result=str(folder/'result.json'),
            report=str(folder/'report.html'),joint_positions=str(folder/'joint_positions.csv'),
            commands=str(folder/'commands.csv'),execution=str(folder/'execution')))


@contextmanager
def _cancellation(instance):
    """Let the existing coordinator and device subprocess finish their own logs."""
    requested=threading.Event();previous={}
    def cancel(*unused):
        requested.set();instance.cancel()
    def check():
        if requested.is_set(): raise backend_module.Cancelled('用户取消方法；后续指令已请求停止')
    try:
        if threading.current_thread() is threading.main_thread():
            for sig in (signal.SIGINT,signal.SIGTERM):
                previous[sig]=signal.signal(sig,cancel)
        yield check
    finally:
        for sig,handler in previous.items(): signal.signal(sig,handler)


def execute_method(path,approval_text,*,mode='live',emit=None):
    if not isinstance(approval_text,str) or not approval_text.strip():
        raise ValueError('执行需要用户实际明确授权文字；哈希不能替代批准')
    if mode=='live':check_live_interface()
    path,payload,review_path=_load_method(path,mode)
    instance=Backend(mode=mode,emit=emit)
    instance.resume_review(review_path)  # Validate everything before the approved startup action.
    claim=Path(str(path)+'.consumed')
    approval=dict(text=approval_text,method_approval_sha256=payload['method_approval_sha256'],
        manifest_sha256=payload['manifest_sha256'],confirmed_at=datetime.now(ZONE).isoformat(),
        startup_actions=payload['startup_actions'],execution_scope=FIXED_PREGRASP)
    with claim.open('x') as stream:
        json.dump(approval,stream,ensure_ascii=False,allow_nan=False);stream.write('\n')
    write_json(path.parent/'method_approval.json',approval)
    cameras=None;startup=None;result={};error=None
    try:
        with _cancellation(instance) as check_cancel:
            check_cancel()
            cameras=instance.start_cameras()
            check_cancel()
            if (cameras.get('completed') is not True or
                    any(cameras.get('cameras',{}).get(role,{}).get('ready') is not True for role in ('wrist','global'))):
                raise RuntimeError('批准的双相机启动及核验未通过：'+str(cameras.get('error') or cameras.get('cameras')))
            startup=instance.control_device('power_on')
            check_cancel()
            if (startup.get('completed') is not True or startup.get('status_fresh') is not True
                    or startup.get('status',{}).get('power_status')!=1
                    or startup.get('status',{}).get('error_status')!=0):
                raise RuntimeError('批准的协议上电及回读未通过：'+str(startup.get('error') or startup.get('message') or startup.get('status')))
            review=instance.resume_review(review_path)
            check_cancel()
            result=instance.execute(review,approval_text,execution_scope=FIXED_PREGRASP)
    except Exception as exc:
        error=str(exc)
        result=dict(status='cancelled' if isinstance(exc,backend_module.Cancelled) else 'failed',
            pregrasp_reached=False,error=error)
    summary=_method_summary(payload,result,startup,error,cameras=cameras)
    summary['execution_metrics']=collect_execution_metrics(payload,summary)
    write_json(path.parent/'method_summary.json',summary)
    return summary


def collect_execution_metrics(payload,summary):
    """Analyze saved evidence after completion or failure; never contact devices."""
    reports=[]
    for stage in summary.get('stage_reports',[]):
        stage_dir=Path(stage['path']).parent
        if not all((stage_dir/name).is_file() for name in ('result.json','plan.json','transport.jsonl')):
            continue
        output=Path(payload['run_dir'])/'analysis'/stage_dir.parent.name
        try:
            tool=backend_module.APP/'tools/analyze_execution.py'
            spec=importlib.util.spec_from_file_location('_d1_closed_loop_metrics',tool)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            metrics,tables=module.analyze(stage_dir)
            module.write_report(output,metrics,tables)
            reports.append(dict(stage=str(stage_dir),metrics=str(output/'metrics.json'),
                                report=str(output/'report.html'),completed=metrics['completed']))
        except Exception as exc:
            reports.append(dict(stage=str(stage_dir),error=str(exc)))
    return reports


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    prepare=commands.add_parser('prepare',help='只读采集并准备方法，输出具体目标、路径、参数和哈希')
    prepare.add_argument('--method',choices=[METHOD],required=True)
    prepare.add_argument('--valid-for-s',type=float,default=7200.)
    prepare.add_argument('--mode',choices=['live','demo'],default='live')
    execute=commands.add_parser('execute',help='消耗一次真实方法批准并自动运行；失败不能重放')
    execute.add_argument('--review',type=Path,required=True)
    execute.add_argument('--approval-text',required=True)
    execute.add_argument('--mode',choices=['live','demo'],default='live')
    args=parser.parse_args(argv)
    def emit(event):
        if event.get('kind') in ('log','state'): print(event.get('message',''),file=sys.stderr,flush=True)
    try:
        if args.command=='prepare':
            payload=prepare_method(args.method,mode=args.mode,valid_for_s=args.valid_for_s,emit=emit)
            print((Path(payload['run_dir'])/'summary.txt').read_text(),end='')
            return 0
        summary=execute_method(args.review,args.approval_text,mode=args.mode,emit=emit)
        print(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False))
        return 0 if summary['completed'] else 1
    except (OSError,ValueError,RuntimeError,KeyError,TypeError) as exc:
        print('方法未执行或未完成：'+str(exc),file=sys.stderr)
        return 1


if __name__=='__main__':
    raise SystemExit(main())
