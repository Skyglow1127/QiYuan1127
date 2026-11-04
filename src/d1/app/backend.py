"""Fixed OpenCV → collision-checked geometric plan → D1 single-cycle workflow."""
from datetime import datetime, timezone, timedelta
from functools import wraps
import fcntl
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
import uuid

from .reporting import load, rows, write_json, export_report, feedback_records, feedback_marker_sample
from .service_errors import is_network_error, saved_service_failure
from .execution_scope import FULL_CYCLE, PREGRASP, FIXED_PREGRASP, select_execution_stages
from .fixed_pose import fixed_motion_checked
from .arm_diagnostics import format_arm_execution_error

APP = Path(__file__).resolve().parents[1]
ARCHIVE = APP.parent
CONTROL = APP/'runtime/Control'
TOOLS = CONTROL/'tools'
ZONE=timezone(timedelta(hours=8))
DEFAULTS=dict(speed_deg_s=.5, acceleration_deg_s2=1., gripper_speed_sdk_s=.5,
              lift_height_m=.10, open_target=36., close_target=29.6, rate_hz=10.,
              retreat_height_m=.08, yaw_deg=90., grasp_height_offset_m=.01)

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def settings_checked(values):
    result=dict(DEFAULTS)
    for key,value in values.items():
        if key in result:
            if isinstance(value,bool): raise ValueError(key+' 必须是数值')
            result[key]=float(value)
    for key,value in result.items():
        if not math.isfinite(value): raise ValueError(key+' 必须是有限数值')
    for key in ('speed_deg_s','acceleration_deg_s2','gripper_speed_sdk_s','rate_hz'):
        if result[key]<=0: raise ValueError(key+' 必须大于 0')
    if result['rate_hz']>10: raise ValueError('通信指令频率不得超过已有传输支持的 10 Hz')
    if not 0<result['lift_height_m']<=.1: raise ValueError('本流程抬升高度应在 0–0.10 米内')
    if not 0<result['retreat_height_m']<=.15: raise ValueError('退离高度应在 0–0.15 米内')
    if not 0<=result['close_target']<result['open_target']<=50: raise ValueError('夹爪目标需要 0 ≤ 闭合位置 < 张开位置 ≤ 50 SDK')
    if result['open_target']-result['close_target']>10: raise ValueError('本次单段夹爪变化不得超过 10 SDK')
    return result


def fixed_endpoint_notices(policy):
    """Describe the exact endpoint retry behavior included in a fixed review."""
    from d1_pick_place.endpoint_control import checked_endpoint_policy
    policy=checked_endpoint_policy(policy)
    if policy is None:return []
    return [(f"固定姿态终点：每 {policy['retry_interval_s']:g} 秒仅向未到位臂轴重发原批准终点，"
             f"不改变目标；偏差 ≤1° 且连续3次新反馈稳定才完成。"
             f"{policy['stall_timeout_s']:g} 秒无 ≥{policy['min_progress_deg']:g}° 改善"
             f"或累计寻位 {policy['max_seek_s']:g} 秒停止；J6 张开目标不变。")]

class Cancelled(RuntimeError): pass

def _exclusive_operation(function):
    """Serialize a Backend's ownership of the process-wide device lease."""
    @wraps(function)
    def wrapped(self,*args,**kwargs):
        if not self._operation_lock.acquire(blocking=False):
            raise RuntimeError('当前程序正在处理设备请求，请等待本次操作结束')
        try:
            return function(self,*args,**kwargs)
        finally:
            self._operation_lock.release()
    return wrapped

def device_status_from_observation(observation,mode='live'):
    """Expose saved feedback with its original age, never as a new live sample."""
    if not isinstance(observation,dict): observation={}
    received=observation.get('snapshot_received_at')
    if not isinstance(received,(int,float)) or isinstance(received,bool) or not math.isfinite(received):
        try: received=datetime.fromisoformat(observation['captured_at']).timestamp()
        except (KeyError,TypeError,ValueError): received=None
    status=observation.get('status')
    status=status if isinstance(status,dict) else {}
    ages=observation.get('status_age_s',{})
    ages=ages if isinstance(ages,dict) else {}
    now=time.time()
    fresh=received is not None and 0<=now-received<=.6
    for key in ('power_status','enable_status','error_status'):
        age=ages.get(key)
        if not isinstance(age,(int,float)) or isinstance(age,bool) or not math.isfinite(age) or not 0<=age<=.6:
            fresh=False
        if type(status.get(key)) is not int or status.get(key) not in (0,1): fresh=False
    return dict(mode=mode,status=status,status_fresh=bool(fresh),snapshot_received_at=received,
                status_age_s=ages,sdk_angles=observation.get('sdk_angles'),
                robot_commands_sent=observation.get('robot_commands_sent',observation.get('hardware_commands_sent',0)))

def _reason(value):
    """Readable structured diagnostics, without dumping arbitrary report JSON."""
    if isinstance(value,dict):
        message=value.get('message') or value.get('error')
        if not isinstance(message,str): return ''
        action=value.get('action')
        readable=_reason(message) if is_network_error(message) or message.startswith('Silhouette fit rejected') else message.strip()
        if is_network_error(message):
            return readable  # A network failure has no captured diagnostic image.
        return readable+('；'+action.strip() if isinstance(action,str) and action.strip() else '')
    if not isinstance(value,str) or not value.strip(): return ''
    value=value.strip()
    prefix='Fresh target recognition failed: '
    if value.startswith(prefix):
        return '执行前目标复核未通过：'+_reason(value[len(prefix):])
    if is_network_error(value):
        cause='连接被拒绝，服务未接受连接' if 'connection refused' in value.lower() else '连接失败或响应超时'
        return '服务访问失败：'+cause+'；请检查服务启动状态及网络连接，恢复后重新准备'
    if value.startswith('Silhouette fit rejected'):
        return value.replace('Silhouette fit rejected','红块轮廓拟合未通过',1)
    return {
        '目标接触图像边界':'目标接触图像边界；查看诊断图，确保红方块完整出现在画面内，再重新准备',
        'Actual joint outside limits':'实际关节位置超出现有限位；检查下面的各轴反馈和范围',
        'Angle feedback stale':'关节反馈缺失或超过 0.6 秒；检查机械臂通信后重新准备',
        'Missing robot feedback':'缺少机械臂关节反馈；检查机械臂通信后重新准备',
        'Insufficient distinct feedback samples':'有效关节反馈样本不足；检查机械臂通信后重新准备',
        'Robot moving during observation':'采集期间关节仍在运动；静止后重新准备',
        'Unexpected wrist camera':'腕部相机身份或运行状态异常；检查腕部相机连接与服务',
        'Wrist frame stale at reception':'腕部画面接收时已过期；检查相机与网络后重新准备',
        'Foreground is occluded or not a convex cube silhouette':'红块分割轮廓有缺口或凹陷；请检查遮挡与颜色分割，当前无法确认目标位姿',
    }.get(value,value)

def _unique(values):
    # Reports repeat a cause with/without its appended action. Keep the first
    # (structured blocker) wording instead of showing the same cause again.
    result=[];seen=set()
    for value in values:
        if not value: continue
        key=value.split('；',1)[0].strip()
        if key not in seen:
            result.append(value);seen.add(key)
    return result

def _report_reasons(report):
    if not isinstance(report,dict): return []
    result=[]
    service_failure=saved_service_failure(report)
    if service_failure:
        result.append(service_failure.message+'；'+service_failure.action)
    for key in ('blockers','error','camera_error','robot_readiness_errors','robot_readiness_error','rejection_reason','rejection_reasons'):
        if key=='robot_readiness_error' and report.get('robot_readiness_errors'):
            continue  # Legacy joined alias of the detailed list.
        value=report.get(key)
        for item in value if isinstance(value,list) else [value]:
            message=item.get('message',item.get('error','')) if isinstance(item,dict) else item
            if service_failure and is_network_error(message): continue
            result.append(_reason(item))
    return _unique(result)

def preparation_notices(q,values,status=None):
    """Explain automatic, reviewable setup without turning its target into a prerequisite."""
    notices=[]
    if (isinstance(q,list) and len(q)==7 and isinstance(q[6],(int,float))
            and not isinstance(q[6],bool) and math.isfinite(q[6]) and 0<=q[6]<=50):
        notices.append(f"夹爪实际 J6={q[6]:.2f} SDK → 抓取前张开目标 {values['open_target']:.2f} SDK；这是批准执行后的首个调整目标，无需手工把初始夹爪调到该位置")
    if isinstance(status,dict) and status.get('enable_status')==0:
        notices.append('enable_status=0 可准备路径；确认执行后，由首个已批准的 SDK 位置动作进入位置控制，并核对实际反馈')
    return notices

def _planning_failure_reasons(output,run_dir):
    """Inspect only fixed planner artifacts inside this subprocess's run directory."""
    root=Path(run_dir).resolve();output=Path(output)
    if not output.resolve().is_relative_to(root): return []
    def contained(path):
        return path.is_file() and path.resolve().is_relative_to(output.resolve())
    reasons=[]
    candidates=[output/'plan.json',*sorted(output.glob('[0-9][0-9]_*/planned/plan.json'))]
    specific=re.compile(
        r'(?:start(?:ing)? state|starting state).*?(?:outside bounds|out of bounds|in collision|collid|violates.*constraints)'
        r'|(?:collision|contact)\s+(?:between|detected|found)'
        r'|CheckStartStateCollision[\'\"]?\s+failed.*?contact(?:\(s\)|s)?\s+detected'
        r'|(?:IK|inverse kinematics).*?(?:fail|no solution|not found|timed out)'
        r'|(?:fail(?:ed)?|unable)\s+to\s+(?:find|compute).*?(?:IK|inverse kinematic)'
        r'|(?:goal.*?in collision|unable to sample any valid states for goal tree|no solution found)',re.IGNORECASE)
    for plan_path in candidates:
        if not contained(plan_path): continue
        report=load(plan_path,{})
        if not isinstance(report,dict) or report.get('completed') is True: continue
        structured=_report_reasons(report)
        if not structured: continue
        stage=str(report.get('stage') or plan_path.parent.parent.name)[:80]
        moveit_log=plan_path.parent/'moveit.log';details=[]
        if contained(moveit_log):
            try:
                with moveit_log.open('rb') as stream:
                    stream.seek(0,2);stream.seek(max(0,stream.tell()-96000))
                    lines=stream.read().decode(errors='replace').splitlines()
                for line in lines:
                    if specific.search(line):
                        # Remove ROS timestamp/node prefixes while retaining the actual cause.
                        detail=re.sub(r'^(?:\[[^\]]*\]\s*)+:?\s*','',line).strip()
                        if 'CheckStartStateCollision' in detail:
                            contacts=re.search(r'contact(?:\(s\)|s)?\s+detected\s*:\s*([^\']+)',detail)
                            pairs=re.findall(r'([\w/]+)\s+-\s+([\w/]+)',contacts.group(1)) if contacts else []
                            if pairs:
                                names='、'.join(left+' 与 '+right for left,right in pairs)
                                detail=f'当前姿态的模型检测到 {names} 碰撞；{detail}'
                        if detail: details.append(f'规划阶段 {stage}：{detail}')
            except OSError: pass
        reasons.extend(_unique(details)[-6:])
        reasons.extend(f'规划阶段 {stage}：{reason}' for reason in structured)
        reasons.append('阶段报告：'+str(plan_path))
        if contained(moveit_log): reasons.append('规划日志：'+str(moveit_log))
    return _unique(reasons)

_STAGE_LABELS = dict(prepare_gripper='抓取前调整夹爪', pregrasp='移动至预抓取点',
                     approach='下降接近抓取点', close='闭合夹爪', lift='抬升',
                     lower='放回', open='张开夹爪', retreat='退离')

def _finite_number(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)

def _executor_reason(report,path):
    """Explain recorded feedback without claiming contact or inventing a cause."""
    error=str(report.get('error') or '')
    failure=report.get('feedback_failure_diagnostics')
    feedback={**report,**failure} if isinstance(failure,dict) else report
    raw_age=feedback.get('final_raw_feedback_age_s')
    rejected=feedback.get('feedback_rejected_count')
    pending=feedback.get('feedback_pending_at_end',feedback.get('feedback_pending'))
    filtered_raw=(pending is True and type(rejected) is int and rejected>0
                  and _finite_number(raw_age) and 0<=raw_age<=.6)
    if error.startswith('All joint feedback recovery timeout'):
        timeout=feedback.get('feedback_gap_timeout_s',2.)
        if not _finite_number(timeout) or timeout<=.6:timeout=2.
        gap_count=feedback.get('all_feedback_gap_count')
        gap_start=feedback.get('last_all_feedback_gap_start_s')
        gap_end=feedback.get('last_all_feedback_gap_end_s')
        if (type(gap_count) is int and gap_count>0 and _finite_number(gap_start)
                and _finite_number(gap_end) and gap_start>=0 and gap_end-gap_start>timeout):
            return (f'执行期间全部 J0–J6 曾连续 {gap_end-gap_start:.3f} 秒无有效反馈，'
                    f'已超过 {timeout:g} 秒；后到帧不能撤销超时；已终止本次执行'
                    '；等待恢复期间未发送新目标；查看逐轴反馈超时记录')
        ages=feedback.get('joint_feedback_age_s')
        detail=''
        if isinstance(ages,list) and len(ages)==7 and all(_finite_number(age) and age>=0 for age in ages):
            detail='；逐轴有效反馈年龄 '+', '.join(f'J{i}={age:.2f}s' for i,age in enumerate(ages))
        return (f'全部 J0–J6 的有效反馈均超过 {timeout:g} 秒且未恢复；已终止本次执行'
                '；等待恢复期间未发送新目标'+detail+'；查看逐轴反馈超时记录')
    if error.startswith('Angle feedback recovery timeout'):
        timeout=feedback.get('feedback_gap_timeout_s',2.)
        if not _finite_number(timeout) or timeout<=.6:timeout=2.
        cause=('原始反馈仍在到达，但连续未通过速度筛选；' if filtered_raw else
               '原始关节反馈已停更；' if _finite_number(raw_age) and raw_age>.6 else '')
        return (cause+f'距最后有效关节反馈已超过 {timeout:g} 秒且未恢复；'
                '已终止本次执行，暂停期间未发送新目标；'
                +('查看筛选诊断' if filtered_raw else '查看反馈恢复记录'))
    if error=='Angle feedback stale' and filtered_raw:
        return '原始反馈仍在到达，但连续未通过速度筛选，有效反馈超过0.6秒；检查筛选诊断'
    q=report.get('final_angles',report.get('actual_sdk'))
    valid_q=isinstance(q,list) and len(q)==7 and all(_finite_number(v) for v in q)
    localized=format_arm_execution_error(error,report.get('execution_diagnostics'))
    if localized!=error:return localized
    if error.startswith(('J6 endpoint not reached/stable','J6 endpoint not stable/within')):
        target=report.get('target_sdk_units')
        tolerance=report.get('endpoint_tolerance_sdk',.5)  # Legacy gripper reports used 0.5 SDK.
        if valid_q and _finite_number(target) and _finite_number(tolerance):
            delta=abs(q[6]-target)
            detail=f'J6 目标 {target:.2f} SDK，最终反馈 {q[6]:.2f} SDK，偏差 {delta:.2f} SDK，允许偏差 ≤{tolerance:.2f} SDK'
            if delta>tolerance+1e-5:
                return '夹爪未到达目标位置：'+detail+'；已停止后续动作，是否夹住物体尚未确认'
            return '夹爪到位检查未通过：'+detail+'；位置偏差在容差内，但稳定性或有效反馈样本条件未满足'
        return '夹爪未满足到位或稳定性要求；指令接收回执不代表动作完成'
    if error.startswith('Endpoint position not reached'):
        plan_path=path.parent/'plan.json'
        if plan_path.is_file() and plan_path.resolve().is_relative_to(path.parent.resolve()):
            plan=load(plan_path,{})
            sdk_plan=plan.get('sdk_plan') if isinstance(plan,dict) else None
            points=sdk_plan.get('sdk',[]) if isinstance(sdk_plan,dict) else []
            tolerance=report.get('endpoint_tolerance_deg')
            target=points[-1] if isinstance(points,list) and points else None
            if (valid_q and isinstance(target,list) and len(target)==6 and
                    all(_finite_number(v) for v in target) and _finite_number(tolerance)):
                deltas=[abs(a-b) for a,b in zip(q[:6],target)]
                offenders=[f'J{i} 目标 {target[i]:.3f}°，最终反馈 {q[i]:.3f}°，偏差 {delta:.3f}°'
                           for i,delta in enumerate(deltas) if delta>tolerance+1e-5]
                if offenders:
                    return f'机械臂未到位（允许偏差 ≤{tolerance:g}°）：'+'；'.join(offenders)
                return f'机械臂到位检查未通过：最终位置偏差均在 {tolerance:g}° 内，但稳定性或有效反馈样本条件未满足'
        return '机械臂未满足终点到位条件；指令接收回执不代表动作完成'
    return {
        'Tracking error >3 degrees':'运动跟踪偏差超过 3°，实际关节位置未跟上轨迹参考；已停止后续指令',
        'J6 tracking error > 3 SDK units':'夹爪运动跟踪偏差超过 3 SDK，已停止后续指令',
        'Receipt timeout; no next command':'等待设备接收回执超时，未发送下一条指令',
        'Missed command schedule; no catch-up burst':'指令发送未赶上计划时刻，已停止发送，避免突发补发',
    }.get(error,_reason(error))

def _execution_failure_reasons(output,run_dir):
    """Read fixed executor result files, never paths embedded in an error message."""
    root=Path(run_dir).resolve();output=Path(output)
    if not output.resolve().is_relative_to(root): return []
    candidates=[]
    for filename in ('summary.json','result.json'):
        candidates.extend(sorted(output.glob('[0-9][0-9]_*/executed/'+filename)))
    reasons=[]
    for path in candidates:
        if not path.is_file() or not path.resolve().is_relative_to(output.resolve()): continue
        report=load(path,{})
        if not isinstance(report,dict) or report.get('completed') is True: continue
        structured=_report_reasons(report)
        explanation=_executor_reason(report,path)
        if explanation:
            structured=[explanation,*[v for v in structured if v!=_reason(report.get('error'))]]
        for segment in report.get('segment_results',[]) if isinstance(report.get('segment_results'),list) else []:
            structured.extend(_report_reasons(segment))
        structured=_unique(structured)
        if not structured: continue
        stage=str(report.get('stage') or path.parent.parent.name)[:80]
        stage_key=re.sub(r'^\d+_','',stage)
        stage_label=_STAGE_LABELS.get(stage_key,stage)
        if any('limit' in reason.lower() or '限位' in reason for reason in structured):
            q=report.get('final_angles',report.get('actual_sdk'))
            if isinstance(q,list) and len(q)==7 and all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in q):
                package=str(CONTROL/'ros2_ws/src/d1_pick_place')
                if package not in sys.path: sys.path.insert(0,package)
                try:
                    from d1_pick_place.sdk_executor import LIMITS
                    # Saved failures retain their original reason even after the live
                    # feedback-tolerance policy changes. Show the recorded value and
                    # nominal bounds; do not reinterpret old execution as successful.
                    for i,(value,(low,high)) in enumerate(zip(q,LIMITS)):
                        if not low-1e-5<=value<=high+1e-5:
                            unit='SDK' if i==6 else '°'
                            reasons.append(f'执行阶段 {stage}：限位失败时 J{i}={value:.3f}{unit}，名义范围 [{low:g}, {high:g}]{unit}；保留当次执行器的限位失败记录')
                except (ImportError,OSError): pass
        reasons.extend(f'执行阶段 {stage_label}：{reason}' for reason in structured)
        reasons.append('执行阶段报告：'+str(path))
    return _unique(reasons)

class ProcessFailure(RuntimeError):
    def __init__(self,argv,code,log_path):
        self.log_path=Path(log_path)
        args=list(map(str,argv));output=None
        for i,arg in enumerate(args):
            if arg=='--output' and i+1<len(args): output=Path(args[i+1])
            elif arg.startswith('--output='): output=Path(arg.split('=',1)[1])
        reasons=[]
        has_stage_reason=False
        if output is not None:
            reasons.extend(_execution_failure_reasons(output,self.log_path.parent))
            has_stage_reason=bool(reasons)
            reasons.extend(_planning_failure_reasons(output,self.log_path.parent))
            for filename in ('result.json','summary.json'):
                for reason in _report_reasons(load(output/filename,{})):
                    if has_stage_reason and re.match(r'^(?:\w+\.py failed \(\d+\); see |Stage did not complete:)',reason):
                        continue
                    reasons.append(reason)
        if not reasons:
            # A traceback/argparse error can precede a long timing JSON. Never
            # replace the failure with unrelated trailing closing braces.
            lines=self.log_path.read_text(errors='replace').splitlines()
            reasons=[line.strip() for line in lines if re.search(
                r'(^[\w.]+(?:Error|Exception):|\berror:|^STOP:|^Error:)',line,re.IGNORECASE)]
            if not reasons and any('Traceback (most recent call last)' in line for line in lines):
                reasons=['子进程发生 Python 异常；请查看完整日志中的 Traceback']
        self.reasons=_unique(reasons) or ['子进程未提供可解析的失败原因；请查看完整日志']
        self.process_name=Path(args[0]).name
        self.exit_code=code
        self.technical_error=f'{self.process_name} 退出 {code}'
        super().__init__('\n'.join(self.reasons)+f'\n完整日志：{self.log_path}')

def preparation_diagnostics(folder,values,exc):
    """Summarize this run's saved evidence even when vision failed first."""
    folder=Path(folder);perception=folder/'perception'
    observation=load(perception/'wrist_observation.json',{})
    if not isinstance(observation,dict) or not observation:
        # Camera acquisition can fail after read-only feedback was saved.
        observation=load(perception/'robot_status.json',{})
    observation=observation if isinstance(observation,dict) else {}
    reasons=[]
    for filename in ('result.json','foreground.json'):
        reasons.extend(_report_reasons(load(perception/filename,{})))
    reasons.extend(_report_reasons(observation))
    reasons=_unique(reasons)
    q=observation.get('sdk_angles')
    if not (isinstance(q,list) and len(q)==7 and all(
            isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in q)):
        q=None
    if q is not None:
        # Read the executor's actual limits; importing definitions sends nothing.
        package=str(CONTROL/'ros2_ws/src/d1_pick_place')
        if package not in sys.path: sys.path.insert(0,package)
        try:
            from d1_pick_place.sdk_executor import feedback_limit_errors
            for reason in feedback_limit_errors(q):
                joint=re.search(r'\bJ\d+',reason)
                if joint and any(joint.group() in prior and ('超出' in prior or '限位' in prior) for prior in reasons):
                    continue
                reasons.append(reason)
        except (ImportError,OSError) as error:
            reasons.append(f'无法读取执行器限位定义：{error}')
    status=observation.get('status')
    if isinstance(status,dict):
        for key,allowed in [('power_status',(1,)),('enable_status',(0,1)),('error_status',(0,))]:
            value=status.get(key)
            if (type(value) is not int or value not in allowed) and not any(key+'=' in reason for reason in reasons):
                required=' 或 '.join(map(str,allowed))
                reasons.append(f'控制软件协议标志 {key}={value!r}，执行器要求 {required}；检查控制器状态后重新准备。该标志不能证明实物供电、使能或故障状态')
    if isinstance(exc,ProcessFailure):
        reasons.extend(exc.reasons)
        logfile=str(exc.log_path)
    else:
        report=load(perception/'result.json',{})
        if not (saved_service_failure(report) and is_network_error(str(exc))):
            reasons.append(_reason(str(exc)))
        logfile=None
    reasons=_unique(reasons)
    if any('超出现有限位 [' in reason for reason in reasons):
        reasons=[reason for reason in reasons if reason!=_reason('Actual joint outside limits')]
    image=next((str(perception/name) for name in ('diagnostic_overlay.jpg','pose_overlay.jpg','wrist.jpg')
                if (perception/name).is_file()),None)
    return dict(blockers=reasons,notices=preparation_notices(q,values,status),
                observed_joint_angles=q,image=image,full_log=logfile)

class Backend:
    def __init__(self, mode='live', emit=None):
        if mode not in ('live','demo'): raise ValueError('Invalid mode')
        self.mode=mode; self.emit=emit or (lambda event:None)
        self.stop=threading.Event(); self.proc=None; self.run_dir=None
        self._lock=threading.Lock(); self._lockfile=None
        self._operation_lock=threading.Lock()
        self._review=None; self._used=set()
        self._active_device_control=None
        self._demo_device_status=dict(power_status=0,enable_status=0,error_status=0)

    def log(self,message): self.emit({'kind':'log','message':message})
    def state(self,message): self.emit({'kind':'state','message':message})
    def cancelled(self):
        if self.stop.is_set(): raise Cancelled('用户停止；不再下发后续指令')

    def _claim_device(self):
        # One process owns preparation/execution at a time, including GUI instances.
        if self._lockfile is not None:
            raise RuntimeError('当前程序已占用设备，请等待本次操作结束')
        path=APP/'logs'/('demo.lock' if self.mode=='demo' else 'device.lock')
        path.parent.mkdir(parents=True,exist_ok=True)
        lockfile=path.open('a+')
        try: fcntl.flock(lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            lockfile.close()
            raise RuntimeError('另一个程序正在使用设备，请等待它结束')
        self._lockfile=lockfile

    def _release_device(self):
        if self._lockfile:
            fcntl.flock(self._lockfile,fcntl.LOCK_UN);self._lockfile.close();self._lockfile=None

    def _run(self,argv,log_path,timeout=1500,monitor_run=True):
        self.cancelled()
        log_path=Path(log_path);log_path.parent.mkdir(parents=True,exist_ok=True)
        write_json(log_path.with_suffix('.argv.json'),[str(v) for v in argv])
        env=dict(os.environ,ROS_DOMAIN_ID='79',PYTHONUNBUFFERED='1')
        with log_path.open('w') as stream:
            proc=subprocess.Popen([sys.executable,*map(str,argv)],stdout=stream,stderr=subprocess.STDOUT,
                                  env=env,start_new_session=True)
            with self._lock: self.proc=proc
            start=time.monotonic(); last_state=None;last_feedback_t=-1
            try:
                while proc.poll() is None:
                    if self.stop.is_set():
                        self._signal_child(proc)
                        raise Cancelled('用户停止；后续指令已请求停止')
                    if time.monotonic()-start>timeout:
                        self._signal_child(proc)
                        raise TimeoutError('阶段处理超时，已请求停止：'+str(argv[0]))
                    if monitor_run and self.run_dir:
                        value=load(self.run_dir/'execution/result.json',{})
                        state=value.get('state')
                        if state and state!=last_state:
                            self.state(state);self.log('执行状态：'+state);last_state=state
                        # Tail the active raw log only; never fabricate feedback from planned points.
                        logs=sorted((self.run_dir/'execution').glob('*/executed/transport.jsonl'))
                        if logs:
                            try:
                                with logs[-1].open('rb') as raw:
                                    first=raw.readline().decode(errors='replace').strip()
                                    config=json.loads(first) if first else {}
                                    raw.seek(0,2);end=raw.tell();raw.seek(max(0,end-24000));tail=raw.read().decode(errors='replace').splitlines()
                                filtered=((config.get('kind')=='feedback_filter' and config.get('enabled') is True)
                                          or any('[feedback-accepted] ' in line or '[feedback-rejected] ' in line
                                                 for line in tail))
                                for line in reversed(tail):
                                    row=json.loads(line)
                                    if row.get('t',0)<=last_feedback_t: break
                                    text=row.get('line','')
                                    if filtered:
                                        if not text.startswith(('[feedback-accepted] ','[feedback-rejected] ')):continue
                                        event=json.loads(text.split(' ',1)[1])
                                        if text.startswith('[feedback-rejected] ') and 'merged_angles' not in event:continue
                                        sample=feedback_marker_sample(event,time.monotonic())
                                        stamp=sample.pop('t')
                                    else:
                                        if '[rx] ' not in text: continue
                                        message=json.loads(text.split('[rx] ',1)[1])
                                        if message.get('address')!=2 or message.get('funcode')!=1:continue
                                        stamp=row['t']
                                        sample=dict(q=[message['data'][f'angle{i}'] for i in range(7)])
                                    if stamp<=last_feedback_t:break
                                    self.emit(dict(kind='joint',received_monotonic_s=stamp,**sample))
                                    last_feedback_t=stamp;break
                            except (OSError,ValueError,KeyError,TypeError): pass
                    self.stop.wait(.15)
                code=proc.wait()
                if code:
                    stream.flush()
                    raise ProcessFailure(argv,code,log_path)
            finally:
                if proc.poll() is None: self._signal_child(proc)
                with self._lock: self.proc=None

    def _signal_child(self,proc):
        if proc.poll() is not None: return
        # Signal the coordinator first, allowing its child cleanup and final logs.
        proc.send_signal(signal.SIGINT)
        try: proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGKILL);proc.wait()

    def cancel(self):
        self.stop.set()
        if self.run_dir and self._active_device_control is None:
            (self.run_dir/'STOP').touch(exist_ok=True)
        self.log('已请求停止后续指令；软件不能确认机械急停，请观察设备状态。')

    def _new_run(self):
        name=datetime.now(ZONE).strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6]
        path=APP/'logs'/name;path.mkdir(parents=True,exist_ok=False)
        return path

    def _preparation_device_status(self,folder):
        perception=Path(folder)/'perception'
        observation=load(perception/'wrist_observation.json',{})
        if not isinstance(observation,dict) or not observation:
            observation=load(perception/'robot_status.json',{})
        if not observation: return None
        result=device_status_from_observation(observation,self.mode)
        self.emit({'kind':'device_status','data':result})
        return result

    @_exclusive_operation
    def start_cameras(self):
        """Explicit camera-only service request, sharing the device operation lease."""
        self._review=None
        self.stop.clear()
        name=datetime.now(ZONE).strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6]
        folder=APP/'logs/camera_services'/name
        self._active_device_control=folder  # Do not put STOP into an old grasp directory.
        result=dict(completed=False,cameras={},robot_commands_sent=0,mode=self.mode,log_dir=str(folder))
        try:
            self._claim_device()
            if self.mode=='demo':
                result.update(completed=True,synthetic=True,checked_at=time.time(),cameras={
                    role:dict(ready=True,label=label,action='simulated',capture_fps=30.,checked_at=time.time())
                    for role,label in [('wrist','腕部相机'),('global','全局相机')]})
            else:
                failure=None
                try:
                    self._run([APP/'app/camera_services.py','--output',folder],
                              folder.with_suffix('.log'),timeout=100,monitor_run=False)
                except Exception as error: failure=error
                raw=load(folder/'result.json',{})
                if isinstance(raw,dict): result.update(raw)
                if failure is not None:
                    result.update(completed=False,error=str(failure))
                elif (result.get('completed') is not True or
                      any(result.get('cameras',{}).get(role,{}).get('ready') is not True for role in ('wrist','global'))):
                    result.update(completed=False,error=result.get('error') or '未收到两路相机均已就绪的检查结果')
        except Exception as error:
            result.update(completed=False,error=str(error))
        finally:
            if self.stop.is_set():
                result.update(completed=False,error='相机启动检查已取消；已启动的相机服务可能继续运行，请重新点击检查状态')
            result.update(log_dir=str(folder),mode=self.mode,robot_commands_sent=0)
            folder.mkdir(parents=True,exist_ok=True)
            write_json(folder/'result.json',result)
            self._active_device_control=None
            self._release_device()
        self.emit({'kind':'camera_status','data':result})
        self.log(('双相机已就绪，请重新准备目标和路径' if result['completed'] else '双相机启动未完成：'+str(result.get('error','')))+'\n相机日志：'+str(folder))
        return result

    @_exclusive_operation
    def shutdown_devices(self):
        """One idle-only shutdown request; report arm and cameras independently."""
        self._review=None
        self.stop.clear()
        name=datetime.now(ZONE).strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6]
        folder=APP/'logs/shutdown_controls'/name
        self._active_device_control=folder
        result=dict(completed=False,mode=self.mode,log_dir=str(folder),power_off={},camera_stop={},
                    robot_commands_sent=0,position_commands_sent=0)
        try:
            self._claim_device()
            if self.mode=='demo':
                self._demo_device_status['power_status']=0
                result.update(synthetic=True,power_off=dict(completed=True,synthetic=True,
                    status=dict(self._demo_device_status),status_fresh=True,snapshot_received_at=time.time(),
                    status_age_s={key:0. for key in self._demo_device_status},robot_commands_sent=0),
                    camera_stop=dict(completed=True,synthetic=True,cameras={role:dict(stopped=True,ready=False)
                        for role in ('wrist','global')}))
            else:
                for key,argv in [
                    ('power_off',[APP/'app/device_control.py','--action','power_off','--output',folder/'power_off']),
                    ('camera_stop',[APP/'app/camera_services.py','--action','stop','--output',folder/'camera_stop'])]:
                    self.cancelled()
                    failure=None
                    try:self._run(argv,folder/(key+'.log'),timeout=100 if key=='camera_stop' else 15,monitor_run=False)
                    except Cancelled:raise
                    except Exception as error:failure=error
                    raw=load(folder/key/'result.json',{})
                    result[key]=raw if isinstance(raw,dict) else {}
                    if failure is not None:
                        result[key].update(completed=False,error=result[key].get('error') or str(failure))
            power=result['power_off'];cameras=result['camera_stop']
            power_ok=power.get('completed') is True and power.get('status',{}).get('power_status')==0
            cameras_ok=cameras.get('completed') is True and all(
                cameras.get('cameras',{}).get(role,{}).get('stopped') is True for role in ('wrist','global'))
            result['completed']=power_ok and cameras_ok
            result['robot_commands_sent']=power.get('robot_commands_sent',0)
            if not result['completed']:
                errors=[]
                if not power_ok:errors.append('机械臂下电：'+str(power.get('error') or '未确认下电'))
                if not cameras_ok:errors.append('双相机关闭：'+str(cameras.get('error') or '未确认两路服务均已关闭'))
                result['error']='\n'.join(errors)
        except Exception as error:
            result.update(completed=False,error=str(error))
        finally:
            folder.mkdir(parents=True,exist_ok=True)
            write_json(folder/'result.json',result)
            self._active_device_control=None
            self._release_device()
        self.log(('设备关闭完成' if result['completed'] else '设备未全部关闭：'+str(result.get('error','')))+'\n关闭日志：'+str(folder))
        return result

    @_exclusive_operation
    def control_device(self,action):
        """Explicit device request, with independent logs and no grasp execution."""
        if action not in ('refresh','power_on','enable'):
            raise ValueError('不支持的设备操作：'+str(action))
        if action!='refresh': self._review=None
        self.stop.clear()
        name=datetime.now(ZONE).strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6]
        folder=APP/'logs/device_controls'/name
        self._active_device_control=folder
        result=dict(action=action,mode=self.mode,completed=False,status={},status_fresh=False,
                    snapshot_received_at=None,sdk_angles=None,robot_commands_sent=0,
                    error=None,log_dir=str(folder))
        failure=None
        try:
            self._claim_device()
            if self.mode=='demo':
                folder.mkdir(parents=True,exist_ok=False)
                if action=='power_on': self._demo_device_status['power_status']=1
                result.update(status=dict(self._demo_device_status),status_fresh=True,
                              snapshot_received_at=time.time(),synthetic=True,
                              status_age_s={key:0. for key in self._demo_device_status},
                              completed=action!='enable')
                if action=='enable':
                    result['error']='使能说明：无需单独打开 enable；确认路径后由首个已批准的 SDK 位置动作进入位置控制，本说明未连接设备或发送指令'
                (folder/'transport.jsonl').write_text('')
            else:
                try:
                    self._run([APP/'app/device_control.py','--action',action,'--output',folder],
                              folder.with_suffix('.log'),timeout=30,monitor_run=False)
                except Exception as exc:
                    failure=exc
                raw=load(folder/'result.json',{})
                if isinstance(raw,dict): result.update(raw)
                result.update(action=action,mode=self.mode,log_dir=str(folder))
                if failure is not None:
                    result['completed']=False
                    if not result.get('error'): result['error']=str(failure)
                elif result.get('completed') is not True:
                    result['completed']=False
                    if not result.get('error'): result['error']='设备操作未报告完成；请查看本次设备日志'
        except Exception as exc:
            result.update(completed=False,error=str(exc))
        finally:
            self._release_device()
            self._active_device_control=None
            folder.mkdir(parents=True,exist_ok=True)
            write_json(folder/'result.json',result)
        self.emit({'kind':'device_status','data':result})
        if isinstance(result.get('sdk_angles'),list):
            self.emit({'kind':'joint','q':result['sdk_angles']})
        self.log(('设备操作完成：' if result['completed'] else '设备操作未完成：')+action+
                 ('；'+str(result['error']) if result.get('error') else '')+'\n设备日志：'+str(folder))
        return result

    @_exclusive_operation
    def prepare(self,settings,*,fixed_pregrasp=False,fixed_motion=None,scheduling_policy=None,valid_for_s=None):
        if scheduling_policy not in (None,'ack_overlap_v1','shared_progress_v1','timed_group_v1'):
            raise ValueError('未知机械臂下发节奏')
        if scheduling_policy is not None and not fixed_pregrasp:
            raise ValueError('实验下发节奏仅适用于独立固定预抓取姿态')
        if valid_for_s is not None:
            if isinstance(valid_for_s,bool) or not isinstance(valid_for_s,(int,float)) or not math.isfinite(valid_for_s) or not 0<valid_for_s<=7200:
                raise ValueError('计划有效期须在 0–7200 秒内')
        values=settings_checked(settings);self.stop.clear();self._review=None
        if fixed_pregrasp: values.update(fixed_motion_checked(fixed_motion))
        self.run_dir=self._new_run();folder=self.run_dir
        clock={'monotonic':time.monotonic(),'unix':time.time()};write_json(folder/'settings.json',values)
        write_json(folder/'preparation_clock.json',clock)
        self.state('正在准备');self.log('日志目录：'+str(folder))
        try:
            self._claim_device()
            if fixed_pregrasp:
                review=self._prepare_fixed_pregrasp(values,folder,scheduling_policy=scheduling_policy,valid_for_s=valid_for_s)
            elif self.mode=='demo': review=self._demo_prepare(values,folder)
            else:
                issues=check_environment()
                if issues: raise RuntimeError('\n'.join(issues))
                self.log('读取新鲜图像和关节反馈；OpenCV 定位 40 mm 红方块。')
                perception=folder/'perception'
                self._run([TOOLS/'prepare_accel_grasp.py','--capture-live','--backend','hsv','--preprocess','host','--output',perception],folder/'perception.log',60)
                report=load(perception/'result.json',{})
                if not report.get('completed') or not report.get('execution_candidate_eligible'):
                    raise RuntimeError(report.get('robot_readiness_error') or report.get('error') or '识别未形成可执行候选')
                pose=load(perception/'pose.json');q=pose['sdk_start'];self.emit({'kind':'joint','q':q})
                notices=preparation_notices(q,values,report.get('status'))
                for notice in notices: self.log(notice)
                self.log('生成抓取前夹爪调整、接近、闭合、抬升、放回、张开、退离路径，检查碰撞和关节范围。')
                argv=[TOOLS/'run_grasp_cycle.py','--pose',perception/'pose.json','--recognition-report',perception/'result.json',
                      '--expected-angles',*q,'--include-pregrasp','--keep-start-orientation','--style','top','--output',folder/'plan']
                for key,flag in [('speed_deg_s','--speed-deg-s'),('acceleration_deg_s2','--acceleration-deg-s2'),
                    ('gripper_speed_sdk_s','--gripper-speed-sdk-s'),('rate_hz','--rate-hz'),('open_target','--open-target'),
                    ('close_target','--close-target'),('lift_height_m','--lift-height-m'),('retreat_height_m','--retreat-height-m'),
                    ('yaw_deg','--yaw-deg'),('grasp_height_offset_m','--grasp-height-offset-m')]: argv.extend([flag,values[key]])
                self._run(argv,folder/'planning.log',1500)
                manifest_path=folder/'plan/manifest.json';manifest=load(manifest_path)
                stages=[]
                for item in manifest['stages']:
                    timing=item.get('sdk_plan',item.get('gripper_plan'))
                    stages.append(dict(item,duration_s=timing['duration_s']))
                review=dict(mode=self.mode,run_dir=str(folder),manifest=str(manifest_path),approval_sha256=digest(manifest_path),
                            target=manifest['target'],parameters=manifest['spec'],stages=stages,
                            expires_at=manifest['expires_at'],ready=True,blockers=[],notices=notices,
                            observed_joint_angles=q,image=str(perception/'pose_overlay.jpg'))
            if not fixed_pregrasp:
                review['notices']=_unique([*review.get('notices',[]),
                    '普通机械臂轨迹也等待反馈接近后才发下一点：J0–J5 与下一目标的最大偏差须≤2.5°；单次准入等待最多2秒、累计最多30秒，不追赶补发。3°跟踪保护针对各轴已实际发布的目标；路径、请求速度及故障阈值不变。'])
            review['device_status']=self._preparation_device_status(folder) or review.get('device_status')
            if review['device_status'] is not None:
                review['notices']=_unique([*review.get('notices',[]),*preparation_notices(
                    review.get('observed_joint_angles'),values,review['device_status'].get('status'))])
            review['settings']=values
            review['preparation_time_s']=time.monotonic()-clock['monotonic']
            write_json(folder/'review.json',review);self._review=json.loads(json.dumps(review))
            self.state('等待确认路径');self.log('路径已准备；确认后自动执行一次，中途无需输入。')
            return review
        except Exception as exc:
            diagnostics=preparation_diagnostics(folder,values,exc)
            if diagnostics['observed_joint_angles'] is not None:
                self.emit({'kind':'joint','q':diagnostics['observed_joint_angles']})
            error='\n'.join(diagnostics['blockers'])
            if diagnostics['full_log']: error+='\n完整日志：'+diagnostics['full_log']
            result=dict(mode=self.mode,status='cancelled' if isinstance(exc,Cancelled) else 'blocked',grasp_result='not_attempted',
                        cycle_result='not_attempted',total_time_s=time.monotonic()-clock['monotonic'],run_dir=str(folder),
                        error=error,robot_commands_sent=0,preparation_diagnostics=diagnostics)
            export_report(folder,result,clock)
            self.emit({'kind':'result','data':result});self.log(error);self.state('准备未通过')
            review=dict(mode=self.mode,run_dir=str(folder),ready=False,**diagnostics,stages=[],parameters=values,
                        settings=values,preparation_time_s=result['total_time_s'],expires_at=0)
            review['device_status']=self._preparation_device_status(folder)
            write_json(folder/'review.json',review);return review
        finally:self._release_device()

    def _prepare_fixed_pregrasp(self,values,folder,*,scheduling_policy=None,valid_for_s=None):
        from d1_pick_place.endpoint_control import ENDPOINT_RETRY_POLICY, checked_endpoint_policy
        adaptive_notice='固定姿态中间参考可按反馈沿相邻已批准参考线段缩小步长；候选偏差保持≤2.5°，原终点及拐点不变，不从实测位置另规划直线。'
        if scheduling_policy=='shared_progress_v1':
            adaptive_notice='各臂轴共享同一轨迹进度；按整臂跟踪裕量协调降速，保留2.5°准入、3°故障及1°终点检查；逐轴ACK仍有提交时差，未声明硬件同步。'
        if scheduling_policy=='timed_group_v1':
            adaptive_notice='整条批准参考及时间轴预缓存在机载，以统一单调时钟下发；正常执行不逐轴等ACK/到位，超期明确失败，终点不重试。'
        profile=load(APP/'app/fixed_ready_pose.json') if (APP/'app/fixed_ready_pose.json').is_file() else load(Path(__file__).with_name('fixed_ready_pose.json'))
        if self.mode=='demo':
            review=self._demo_prepare(values,folder)
            q=review['stages'][1]['expected_angles'].copy()
            target=[math.degrees(v) for v in profile['model_target_rad']]
            stages=[review['stages'][0],dict(name=FIXED_PREGRASP,expected_angles=q,gripper_target=values['open_target'],duration_s=.45,
                progress_policy='scheduled' if scheduling_policy=='timed_group_v1' else 'feedback_paced',
                endpoint_policy=None if scheduling_policy=='timed_group_v1' else checked_endpoint_policy(ENDPOINT_RETRY_POLICY),
                sdk_plan=dict(sdk=[q[:6],target],duration_s=.45,max_velocity_deg_s=values['speed_deg_s']))]
            policy=None if scheduling_policy=='timed_group_v1' else checked_endpoint_policy(ENDPOINT_RETRY_POLICY)
            if valid_for_s is not None: review['expires_at']=time.time()+valid_for_s
            write_json(review['manifest'],dict(mode='demo',operation=FIXED_PREGRASP,stages=stages,parameters=values,
                endpoint_policy=policy,scheduling_policy=scheduling_policy,expires_at=review['expires_at'],
                startup_actions=['start_cameras_if_needed','power_on_if_needed'] if scheduling_policy in ('ack_overlap_v1','shared_progress_v1','timed_group_v1') else []))
            review.update(stages=stages,operation=FIXED_PREGRASP,approval_sha256=digest(review['manifest']),
                scheduling_policy=scheduling_policy,
                startup_actions=['start_cameras_if_needed','power_on_if_needed'] if scheduling_policy in ('ack_overlap_v1','shared_progress_v1','timed_group_v1') else [],
                endpoint_policy=policy,notices=[*review.get('notices',[]),adaptive_notice,*fixed_endpoint_notices(policy)],
                target=dict(description='Gazebo 固定准备姿态（ready）；演示，无硬件动作',model_target_rad=profile['model_target_rad'],sdk_target_deg=target,source=profile['source']))
            return review
        issues=check_environment()
        if issues: raise RuntimeError('\n'.join(issues))
        self.log('读取静止关节反馈，准备 Gazebo ready 固定姿态；不识别红块。')
        observation=folder/'fixed_pose_observation'
        observe_command=[TOOLS/'observe_grasp.py','--allow-position-start','--output',observation]
        if scheduling_policy in ('ack_overlap_v1','shared_progress_v1','timed_group_v1'):
            observe_command.extend(['--allow-unpowered-observation','--arm-only-observation'])
        self._run(observe_command,folder/'observation.log',30)
        observed=load(observation/'wrist_observation.json');q=observed['sdk_angles']
        self.emit({'kind':'joint','q':q})
        reference=CONTROL/'logs/first_grasp_execution_20261022/at_pregrasp/cube_pose.json'
        argv=[TOOLS/'run_grasp_cycle.py','--fixed-ready','--pose',reference,'--expected-angles',*q,'--output',folder/'plan']
        if scheduling_policy is not None: argv.extend(['--scheduling-policy',scheduling_policy])
        if valid_for_s is not None: argv.extend(['--valid-for-s',valid_for_s])
        for key,flag in [('speed_deg_s','--speed-deg-s'),('acceleration_deg_s2','--acceleration-deg-s2'),
            ('gripper_speed_sdk_s','--gripper-speed-sdk-s'),('rate_hz','--rate-hz'),
            ('open_target','--open-target'),('close_target','--close-target'),('lift_height_m','--lift-height-m')]:
            argv.extend([flag,values[key]])
        self._run(argv,folder/'planning.log',300)
        path=folder/'plan/manifest.json';manifest=load(path)
        return dict(mode=self.mode,operation=FIXED_PREGRASP,run_dir=str(folder),manifest=str(path),approval_sha256=digest(path),
            scheduling_policy=scheduling_policy,
            startup_actions=['start_cameras_if_needed','power_on_if_needed'] if scheduling_policy in ('ack_overlap_v1','shared_progress_v1','timed_group_v1') else [],
            target=manifest['target'],parameters=manifest['spec'],
            endpoint_policy=checked_endpoint_policy(manifest.get('endpoint_policy')),
            stages=[dict(item,duration_s=item.get('sdk_plan',item.get('gripper_plan'))['duration_s']) for item in manifest['stages']],
            expires_at=manifest['expires_at'],ready=True,blockers=[],observed_joint_angles=q,
            device_status=device_status_from_observation(observed),
            notices=['固定六轴目标来自 Gazebo ready；不执行红块识别、下探或抓放。',
                     adaptive_notice,
                     ('批准的参考及时间轴交给机载共同调度；超期或整臂跟踪异常明确失败，终点不重发。' if scheduling_policy=='timed_group_v1' else
                      '固定姿态按实际反馈推进，速度参数为参考上限；未跟进超过2秒或累计等待超过30秒停止，不补发积压轨迹。'),
                     '夹爪使用本界面张开目标 SDK 值，不是从仿真开口换算；碰撞场景沿用已标定静态桌面与机械臂模型。',
                     *fixed_endpoint_notices(manifest.get('endpoint_policy'))])

    @_exclusive_operation
    def resume_review(self,path):
        """Restore an unused review produced in this application's own log directory."""
        self._review=None
        path=Path(path).expanduser().resolve(strict=True)
        root=(APP/'logs').resolve()
        folder=path.parent
        if path.name!='review.json' or folder.parent!=root:
            raise ValueError('只能恢复本程序 logs 目录内已保存的 review.json')
        review=load(path)
        if not isinstance(review,dict) or review.get('ready') is not True or review.get('mode')!=self.mode:
            raise ValueError('计划模式不匹配或准备未通过')
        if Path(review.get('run_dir','')).resolve()!=folder:
            raise ValueError('计划日志目录与 review.json 不匹配')
        manifest=Path(review.get('manifest','')).resolve(strict=True)
        expected=folder/('manifest.json' if self.mode=='demo' else 'plan/manifest.json')
        if manifest!=expected.resolve() or not manifest.is_relative_to(folder):
            raise ValueError('计划 manifest 不在原日志目录中')
        if digest(manifest)!=review.get('approval_sha256'):
            raise ValueError('计划文件发生变化，请重新准备')
        expires=review.get('expires_at')
        if isinstance(expires,bool) or not isinstance(expires,(int,float)) or not math.isfinite(expires) or time.time()>expires:
            raise ValueError('计划已过期，请重新准备')
        if (Path(str(manifest)+'.consumed').exists() or (folder/'user_approval.json').exists()
                or (folder/'execution').exists() or review['approval_sha256'] in self._used):
            raise ValueError('本计划已使用，每次需重新准备')
        saved=load(manifest)
        if not isinstance(saved,dict): raise ValueError('无效计划 manifest')
        if self.mode=='demo':
            if (saved.get('mode')!='demo' or saved.get('stages')!=review.get('stages')
                    or saved.get('parameters')!=review.get('parameters')
                    or saved.get('operation')!=review.get('operation')
                    or saved.get('scheduling_policy')!=review.get('scheduling_policy')
                    or saved.get('startup_actions',[])!=review.get('startup_actions',[])
                    or ('expires_at' in saved and saved['expires_at']!=expires)):
                raise ValueError('演示计划与保存的 review 不一致')
        else:
            if saved.get('mode')=='demo': raise ValueError('演示计划不能恢复为实机计划')
            for directory in (TOOLS,CONTROL/'ros2_ws/src/d1_pick_place'):
                if str(directory) not in sys.path: sys.path.insert(0,str(directory))
            spec=importlib.util.spec_from_file_location('_d1_resume_grasp_cycle',TOOLS/'run_grasp_cycle.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            saved=module.validate_manifest(manifest,review['approval_sha256'])
            if str((TOOLS/'run_grasp_cycle.py').resolve()) not in saved.get('files',{}):
                raise ValueError('计划未绑定本程序的执行代码，请重新准备')
            stages=[dict(item,duration_s=item.get('sdk_plan',item.get('gripper_plan'))['duration_s']) for item in saved['stages']]
            if (saved.get('replay_only') or saved.get('execution_candidate_eligible') is not True
                    or saved.get('spec')!=review.get('parameters') or saved.get('target')!=review.get('target')
                    or stages!=review.get('stages') or saved.get('expires_at')!=expires
                    or saved.get('initial_angles')!=review.get('observed_joint_angles')
                    or saved.get('operation')!=review.get('operation')
                    or saved['spec'].get('scheduling_policy','ack_then_period_v1')!=(review.get('scheduling_policy') or 'ack_then_period_v1')
                    or review.get('startup_actions',[])!=(['start_cameras_if_needed','power_on_if_needed'] if review.get('scheduling_policy') in ('ack_overlap_v1','shared_progress_v1','timed_group_v1') else [])):
                raise ValueError('保存的 review 与可执行 manifest 不一致')
        self.run_dir=folder;self._review=json.loads(json.dumps(review))
        return review

    def _validate_review(self,review,approval_text):
        if not isinstance(approval_text,str) or not approval_text.strip(): raise ValueError('需要用户对显示路径的明确确认')
        if self._review is None or review!=self._review or not review.get('ready'): raise ValueError('当前计划未准备或已被修改，请重新准备')
        if review['mode']!=self.mode: raise ValueError('演示计划不能执行实机')
        if time.time()>review['expires_at']: raise ValueError('计划已过期，请重新准备')
        path=Path(review['manifest'])
        if not path.resolve().is_relative_to(self.run_dir.resolve()) or digest(path)!=review['approval_sha256']: raise ValueError('计划文件发生变化，请重新准备')
        if review['approval_sha256'] in self._used: raise ValueError('本计划已使用，每次需重新准备')

    @_exclusive_operation
    def execute(self,review,approval_text,*,execution_scope=FULL_CYCLE):
        self._validate_review(review,approval_text)
        if (review.get('operation')==FIXED_PREGRASP) != (execution_scope==FIXED_PREGRASP):
            raise ValueError('固定姿态计划与完整抓放不能混用，请重新准备对应路径')
        selected_stages=select_execution_stages(review.get('stages',[]),execution_scope)
        self._used.add(review['approval_sha256']);self.stop.clear()
        folder=Path(review['run_dir']);self.run_dir=folder
        clock={'monotonic':time.monotonic(),'unix':time.time()};write_json(folder/'execution_clock.json',clock)
        write_json(folder/'user_approval.json',dict(text=approval_text,manifest_sha256=review['approval_sha256'],confirmed_at=datetime.now(ZONE).isoformat(),parameters=review['parameters'],
                   execution_scope=execution_scope,requested_stages=[stage['name'] for stage in selected_stages]))
        result=dict(mode=self.mode,status='failed',grasp_result='unknown',run_dir=str(folder),stage_times={},
                    physical_stop_verified=False,execution_scope=execution_scope)
        try:
            self._claim_device();self.state('正在进入固定预抓取姿态' if execution_scope==FIXED_PREGRASP else '正在进入预抓取状态' if execution_scope==PREGRASP else '执行中')
            if self.mode=='demo': self._demo_execute(folder,clock,review,execution_scope=execution_scope)
            else:
                command=[TOOLS/'run_grasp_cycle.py','--execute','--manifest',review['manifest'],
                    '--approval-sha256',review['approval_sha256'],'--approval-text',approval_text,
                    '--stop-file',folder/'STOP','--output',folder/'execution']
                if execution_scope in (PREGRASP,FIXED_PREGRASP): command.extend(['--execution-scope',execution_scope])
                self._run(command,folder/'execution.log',7200)
            raw=load(folder/'execution/result.json',{})
            if execution_scope in (PREGRASP,FIXED_PREGRASP):
                if (raw.get('completed') is not True or raw.get('pregrasp_reached') is not True
                        or raw.get('execution_scope')!=execution_scope
                        or raw.get('completed_stages')!=[stage['name'] for stage in selected_stages]
                        or raw.get('trajectory_completed') is not False):
                    raise RuntimeError('执行器未确认已到达预抓取点；请查看阶段报告与实际反馈')
                result.update(status='pregrasp_completed',grasp_result='not_attempted',cycle_result='not_attempted',
                    pregrasp_reached=True,trajectory_completed=False,final_angles=raw.get('final_angles'),details=raw,
                    completion_note='已完成夹爪调整和预抓取移动，到位后停止。继续抓放需从当前位置重新准备并确认路径。')
                if execution_scope==FIXED_PREGRASP:
                    result['completion_note']='已完成夹爪调整，进入 Gazebo ready 固定准备姿态后停止。继续抓放需重新识别并准备路径。'
                return result
            if raw.get('completed') is not True or raw.get('trajectory_completed') is not True:
                raise RuntimeError('执行器未报告完整完成；不得把进程退出当作抓取成功')
            if self.mode == 'live' and (raw.get('grasp_result') != 'success' or raw.get('cycle_result') != 'success'):
                raise RuntimeError('执行器的视觉结果尚未确认完整抓放成功')
            result.update(status='demo_completed' if self.mode=='demo' else 'completed',
                grasp_result=raw.get('grasp_result','unknown'),cycle_result=raw.get('cycle_result','unknown'),
                trajectory_completed=bool(raw.get('trajectory_completed')),details=raw)
        except Exception as exc:
            raw=load(folder/'execution/result.json',{})
            status='cancelled' if self.stop.is_set() or isinstance(exc,Cancelled) else 'failed'
            if status != 'cancelled' and (raw.get('cycle_result') == 'unknown' or (raw.get('grasp_result') == 'unknown' and raw.get('visual_verifications'))):
                status='unconfirmed'
            result.update(status=status,error=str(exc),
                          grasp_result=raw.get('grasp_result','unknown'),cycle_result=raw.get('cycle_result','not_completed'),
                          trajectory_completed=bool(raw.get('trajectory_completed')),details=raw)
            if execution_scope in (PREGRASP,FIXED_PREGRASP):
                result.update(pregrasp_reached=False,grasp_result='not_attempted',cycle_result='not_attempted')
            if isinstance(exc,ProcessFailure):
                result['technical_error']=exc.technical_error
                result['exit_code']=exc.exit_code
            self.log(str(exc))
        finally:
            self._release_device();result['total_time_s']=time.monotonic()-clock['monotonic']
            result['preparation_time_s']=review.get('preparation_time_s')
            export_report(folder,result,clock)
            self.emit({'kind':'result','data':result});self.state(result['status'])
        return result

    def _demo_prepare(self,values,folder):
        self.log('演示模式：使用模拟关节与图像结果，不连接设备。')
        q=[0.,-20.,40.,0.,20.,0.,6.5];initial=q.copy();stages=[]
        for name,delta in [('prepare_gripper',0),('pregrasp',3),('approach',2),('close',0),('lift',-4),('lower',4),('open',0),('retreat',-2)]:
            item=dict(name=name,expected_angles=q.copy(),duration_s=.45)
            if name in ('prepare_gripper','close','open'):
                target=values['close_target'] if name=='close' else values['open_target']
                item['gripper_plan']={'duration_s':.45,'target_sdk_units':target};q[6]=target
            else:
                end=q[:6].copy();end[1]+=delta
                item['sdk_plan']={'sdk':[q[:6].copy(),end],'duration_s':.45,'max_velocity_deg_s':values['speed_deg_s']};q[:6]=end
            stages.append(item)
        manifest=folder/'manifest.json';write_json(manifest,dict(mode='demo',stages=stages,parameters=values))
        return dict(mode='demo',run_dir=str(folder),manifest=str(manifest),approval_sha256=digest(manifest),target={'description':'模拟 40 mm 红方块；无硬件动作'},
                    parameters=values,stages=stages,expires_at=time.time()+600,ready=True,blockers=[],
                    observed_joint_angles=initial,notices=preparation_notices(initial,values,self._demo_device_status))

    def _demo_execute(self,folder,clock,review,*,execution_scope=FULL_CYCLE):
        execution=folder/'execution';execution.mkdir()
        selected_stages=select_execution_stages(review['stages'],execution_scope)
        with (execution/'events.jsonl').open('w') as events:
            for number,stage in enumerate(selected_stages):
                self.cancelled();self.state(stage['name']);events.write(json.dumps({'time':time.time(),'state':'executing','stage':stage['name']})+'\n');events.flush()
                target=execution/f"{number:02d}_{stage['name']}"/'executed';target.mkdir(parents=True)
                with (target/'transport.jsonl').open('w') as log:
                    q=stage['expected_angles'].copy()
                    for j in range(5):
                        self.cancelled()
                        values=q.copy()
                        if 'sdk_plan' in stage: values[:6]=[a+(b-a)*j/4 for a,b in zip(q[:6],stage['sdk_plan']['sdk'][-1])]
                        else: values[6]=q[6]+(stage['gripper_plan']['target_sdk_units']-q[6])*j/4
                        message={'address':2,'funcode':1,'data':{f'angle{i}':v for i,v in enumerate(values)}}
                        log.write(json.dumps({'kind':'output','t':time.monotonic(),'line':'[rx] '+json.dumps(message),'synthetic':True})+'\n');log.flush()
                        self.emit({'kind':'joint','q':values});self.stop.wait(.09)
                events.write(json.dumps({'time':time.time(),'state':'stage_completed','stage':stage['name']})+'\n');events.flush()
        self.cancelled()
        pregrasp_only=execution_scope in (PREGRASP,FIXED_PREGRASP)
        write_json(execution/'result.json',dict(completed=True,trajectory_completed=not pregrasp_only,
            execution_scope=execution_scope,pregrasp_reached=pregrasp_only,completed_stages=[stage['name'] for stage in selected_stages],
            final_angles=values,grasp_result='not_attempted' if pregrasp_only else 'simulated_success',
            cycle_result='not_attempted' if pregrasp_only else 'simulated_success',robot_commands_sent=0))

def check_environment():
    issues=[]
    for name in ('tkinter','cv2','numpy','scipy','rclpy','moveit_msgs','urdf_parser_py','ament_index_python'):
        try: found=importlib.util.find_spec(name)
        except (ValueError,ImportError): found=None
        if found is None: issues.append('缺少运行依赖：'+name+'；请使用启动程序加载归档 ROS 环境')
    for path in [CONTROL/'build/arm_control',CONTROL/'host_build/arm_control',CONTROL/'ros2_ws/src/d1_pick_place/config/d1.urdf',
                 CONTROL/'logs/handeye_resume_20261022_145044/empirical_handeye_candidate.json',
                 CONTROL/'logs/first_grasp_execution_20261022/at_pregrasp/cube_pose.json']:
        if not path.is_file():issues.append('缺少归档文件：'+str(path))
    return issues
