"""Bounded, feedback-checked arm trajectories through the existing D1 CLI.

No power, enable, zero or release commands. SDK J6 holds its measured starting
position or an explicit existing grip target. Receipt and arrival are separate.
"""
from dataclasses import dataclass
import json
import math
from contextlib import nullcontext
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np

from . import path_timing
from .endpoint_control import checked_endpoint_policy
from .adaptive_reference import admissible_reference

LIMITS = np.array([[-135,135],[-90,90],[-90,90],[-135,135],[-90,90],[-135,135],[0,65]])
FEEDBACK_JOINT_TOLERANCE_DEG = .2
ARM_POSITION_TOLERANCE_DEG = 1.
FEEDBACK_FRESHNESS_S = .6
FEEDBACK_RECOVERY_TIMEOUT_S = 2.
FIXED_PROGRESS_MARGIN_DEG = 2.5
NAMES = [f'Joint{i}' for i in range(1,7)]
HOST_BINARY = Path(__file__).resolve().parents[4] / 'host_build/arm_control'


def feedback_limit_errors(q):
    """User-approved arm readback tolerance; command and J6 limits stay fixed."""
    if q is None or np.asarray(q).shape != (7,):
        return ['缺少完整 J0–J6 实际反馈']
    errors=[]
    for joint,(value,(low,high)) in enumerate(zip(q,LIMITS)):
        tolerance=FEEDBACK_JOINT_TOLERANCE_DEG if joint<6 else 0.
        unit='°' if joint<6 else ' SDK'
        if not math.isfinite(value):
            errors.append(f'J{joint} 反馈不是有限数值')
        elif not low-tolerance-1e-5<=value<=high+tolerance+1e-5:
            errors.append(f'J{joint} 实际反馈 {value:.3f}{unit} 超出范围 [{low:g}, {high:g}]{unit}及反馈容差 {tolerance:g}{unit}')
    return errors


def nominal_planning_start(measured):
    """Explicit bounded model reference, never a replacement for raw feedback.

    Only readings within the approved 0.2-degree uncertainty are eligible.
    Callers record measured and nominal values separately; targets stay bounded.
    """
    errors=feedback_limit_errors(measured)
    if errors:raise ValueError('; '.join(errors))
    reference=np.asarray(measured,dtype=float).copy()
    reference[:6]=np.clip(reference[:6],LIMITS[:6,0],LIMITS[:6,1])
    return reference


class Cancelled(Exception):
    pass


@dataclass
class Mapping:
    signs: np.ndarray
    offsets: np.ndarray

    @classmethod
    def from_candidate(cls, path):
        c=json.loads(Path(path).read_text())
        m=c['mapping']
        if m['order'] != [f'J{i}->Joint{i+1}' for i in range(6)]:
            raise ValueError('Unsupported joint order')
        signs=np.array(m['signs'],dtype=float); offsets=np.deg2rad(m['zero_offsets_deg'])
        if signs.shape!=(6,) or offsets.shape!=(6,) or not np.isin(signs,[-1,1]).all() or not np.isfinite(offsets).all():
            raise ValueError('Invalid mapping')
        return cls(signs,offsets)

    def to_model(self, sdk):
        return self.signs*np.deg2rad(np.asarray(sdk)[:6])+self.offsets

    def to_sdk(self, model):
        return np.rad2deg((np.asarray(model)-self.offsets)/self.signs)


def prepare(joint_names, positions, times, mapping, rate=10., speed=.5, acceleration=1.,
            timing_profile=path_timing.PROFILE, respect_source_timing=True):
    """Preserve planned geometry; stop only at actual corners or stage ends.

    Speed/acceleration units are deg/s and deg/s², not vendor specifications.
    Timing is rounded to controller ticks so no catch-up burst is needed.
    Set respect_source_timing=False to retime the collision-checked geometry
    with the requested speed instead of inheriting an older slow trajectory.
    """
    if len(joint_names)!=6 or set(joint_names)!=set(NAMES):raise ValueError('Exactly six named arm joints required')
    q=np.asarray(positions,dtype=float); ts=np.asarray(times,dtype=float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or ts.shape!=(len(q),) or not np.isfinite(q).all() or not np.isfinite(ts).all():
        raise ValueError('Malformed trajectory')
    if ts[0]<0 or np.any(np.diff(ts)<=0):raise ValueError('Trajectory times must increase')
    if any(isinstance(x, bool) or not math.isfinite(x) or x <= 0
           for x in (rate, speed, acceleration)):
        raise ValueError('Rate, speed and acceleration must be positive finite numbers')
    if rate > 10:
        raise ValueError('Command rate must be at most 10 Hz for this feedback transport')
    if not isinstance(respect_source_timing, bool):
        raise ValueError('respect_source_timing must be a boolean')
    if not respect_source_timing:
        # Geometry points are not command ticks. A dense collinear path must
        # not impose one old-duration lower bound per intermediate waypoint.
        # The retimer itself keeps at least one tick for each run or dwell.
        ts = np.zeros(len(q), dtype=float)
    ordered=q[:,[joint_names.index(n) for n in NAMES]]
    sdk=np.array([mapping.to_sdk(row) for row in ordered])
    if np.any(sdk<LIMITS[:6,0]-1e-6) or np.any(sdk>LIMITS[:6,1]+1e-6):raise ValueError('Mapped SDK joint limit exceeded')
    if np.max(np.abs(np.diff(sdk,axis=0)))>20:raise ValueError('Discontinuous planner path')
    if timing_profile not in (path_timing.PROFILE, 'edge_quintic_v1'):
        raise ValueError('Unsupported timing profile')
    common = dict(sdk=sdk.tolist(),rate=rate,
                  max_velocity_deg_s=speed,max_acceleration_deg_s2=acceleration,
                  feedback_joint_tolerance_deg=FEEDBACK_JOINT_TOLERANCE_DEG,
                  command_policy='changed_active_joints_with_mode1_delay_v1',
                  respect_source_timing=respect_source_timing,
                  constraint_basis='user requested reference speed and acceleration; not vendor rated capability',
                  active_joints=np.flatnonzero(np.ptp(sdk,axis=0)>.001).tolist())
    if timing_profile == path_timing.PROFILE:
        return dict(**common, **path_timing.retime(sdk, ts, rate, speed, acceleration))
    ticks=[]
    for i,delta in enumerate(np.max(abs(np.diff(sdk,axis=0)),axis=1)):
        # max derivative of 10u³-15u⁴+6u⁵ is1.875; max second is10/sqrt(3).
        seconds=max(float(ts[i+1]-ts[i]),1.875*delta/speed,math.sqrt((10/math.sqrt(3))*delta/acceleration),1/rate)
        ticks.append(math.ceil(seconds*rate-1e-9))
    return dict(**common,ticks=ticks,duration_s=sum(ticks)/rate,
                timing_profile='edge_quintic_v1')


def references(plan):
    if plan.get('timing_profile') == path_timing.PROFILE:
        yield from path_timing.references(plan)
        return
    if plan.get('timing_profile', 'edge_quintic_v1') != 'edge_quintic_v1':
        raise ValueError('Unsupported timing profile')
    q=np.array(plan['sdk']);step=0
    yield 0.,q[0]
    for a,b,count in zip(q[:-1],q[1:],plan['ticks']):
        for i in range(1,count+1):
            u=i/count;step+=1
            yield step/plan['rate'],a+(b-a)*(10*u**3-15*u**4+6*u**5)


class CliTransport:
    """Persistent CLI, with a reader that keeps feedback fresh while ROS spins."""
    def __init__(self,binary,nic,output,extra_args=(),*,allow_position_start=False,
                 feedback_max_speed_deg_s=None):
        if feedback_max_speed_deg_s is not None and (
                isinstance(feedback_max_speed_deg_s,bool)
                or not math.isfinite(feedback_max_speed_deg_s)
                or feedback_max_speed_deg_s<=0):
            raise ValueError('Feedback maximum speed must be positive finite degrees/second')
        self.lock=threading.RLock();self.changed=threading.Condition(self.lock)
        self.q=None;self.q_time=-math.inf;self.q_seq=0;self.status={};self.status_time={}
        self.joint_q_time=[-math.inf]*7;self.joint_q_seq=[0]*7
        self.joint_feedback_pending=[False]*7
        self.feedback_gap_history_available=False;self.all_feedback_gap_count=0
        self.last_all_feedback_gap_start_s=None;self.last_all_feedback_gap_end_s=None
        self.feedback_filter_enabled=feedback_max_speed_deg_s is not None
        self.feedback_pending=False;self.feedback_pause_total_s=0.;self.feedback_rejected_count=0
        self.raw_q=None;self.raw_q_time=-math.inf
        self.acks=0;self.commands=0;self.fault=None;self.closed=False
        self.allow_position_start=allow_position_start
        self.last_targets={}
        self.published_targets={}
        self.now=time.monotonic
        self.log=Path(output).open('x')
        self.log.write(json.dumps(dict(t=self.now(),kind='feedback_filter',
            enabled=self.feedback_filter_enabled,max_speed_deg_s=feedback_max_speed_deg_s,
            multiplier=3.,scope='J0-J5',acceptance_scope='per_joint',
            timeout_policy='all_joints_stale',rejected_samples_refresh_valid_time=False))+'\n')
        self.log.flush()
        start_args=['--bootstrap-position-once'] if allow_position_start else []
        filter_args=(['--feedback-max-speed',str(feedback_max_speed_deg_s),
                      '--feedback-recovery-timeout-ms',str(int(FEEDBACK_RECOVERY_TIMEOUT_S*1000))]
                     if self.feedback_filter_enabled else [])
        self.proc=subprocess.Popen([str(binary),'--nic',nic,'--joints','all','--ack','recv',
            '--timeout-ms','500','--verbose',*start_args,*filter_args,*extra_args],stdin=subprocess.PIPE,stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,text=True,bufsize=1)
        self.reader=threading.Thread(target=self._read,daemon=True);self.reader.start()

    def _read(self):
        try:
            for line in self.proc.stdout:
                now=self.now();line=line.rstrip()
                with self.changed:
                    self.log.write(json.dumps(dict(t=now,kind='output',line=line))+'\n');self.log.flush()
                    if '[rx] ' in line:
                        m=json.loads(line.split('[rx] ',1)[1]);d=m.get('data',{})
                        if m.get('address')==2 and m.get('funcode')==1:
                            raw=np.array([float(d[f'angle{i}']) if type(d.get(f'angle{i}')) in (int,float)
                                          else math.nan for i in range(7)])
                            self.raw_q=raw;self.raw_q_time=now
                            if not getattr(self,'feedback_filter_enabled',False):
                                self.q=raw.copy();self.q_time=now;self.q_seq+=1
                                self.joint_q_time=[now]*7;self.joint_q_seq=[self.q_seq]*7
                        if m.get('address')==2 and m.get('funcode')==3:
                            for key in ('power_status','enable_status','error_status'):
                                if key in d:self.status[key]=d[key];self.status_time[key]=now
                        if (m.get('address')==2 and m.get('funcode')==90
                                and d.get('schema')=='d1_uart_integrity_v1'):
                            self.onboard_diagnostics=dict(d)
                            self.onboard_diagnostics_time=now
                            # Receipt ACK is upstream of the serial write. A
                            # later board rejection must stop further targets.
                            if (d.get('event')=='command_rejected'
                                    or d.get('event')=='command_write' and d.get('write_ok') is not True):
                                self.fault=self.fault or ('Onboard command rejected: '+str(d.get('reason','unknown')))
                    if line.startswith(('[feedback-accepted] ','[feedback-rejected] ')):
                        if not getattr(self,'feedback_filter_enabled',False):
                            raise RuntimeError('Unexpected feedback filter event')
                        event=json.loads(line.split(' ',1)[1])
                        stamp=float(event['received_monotonic_s'])
                        pause=float(event['pause_total_s'])
                        if (not math.isfinite(stamp) or stamp<0 or stamp>now+.01
                                or not math.isfinite(pause)
                                or pause<self.feedback_pause_total_s-1e-8):
                            raise RuntimeError('Invalid feedback filter event timing')
                        self.feedback_pause_total_s=pause
                        if 'joint_times_s' in event:
                            self._joint_feedback_event(event,stamp)
                        elif line.startswith('[feedback-rejected] '):
                            self.feedback_pending=True;self.feedback_rejected_count+=1
                            self.joint_feedback_pending=[True]*7
                        else:
                            message=event['message'];data=message['data']
                            if message.get('address')!=2 or message.get('funcode')!=1:
                                raise RuntimeError('Invalid accepted feedback message')
                            q=np.array([float(data[f'angle{i}']) for i in range(7)])
                            if not np.isfinite(q).all() or stamp<self.q_time:
                                raise RuntimeError('Invalid accepted angle feedback')
                            self.q=q;self.q_time=stamp;self.q_seq+=1
                            self.feedback_pending=False
                            self.joint_q_time=[stamp]*7;self.joint_q_seq=[self.q_seq]*7
                            self.joint_feedback_pending=[False]*7
                    if line.startswith('[ok] '):self.acks+=1
                    if line.startswith('[tx] '):
                        wire=json.loads(line[5:])
                        if wire.get('address')==1 and wire.get('funcode')==1:
                            joint=wire['data']['id']
                            if type(joint) is not int or not 0<=joint<7:
                                raise RuntimeError('Invalid published joint target')
                            if not hasattr(self,'published_targets'):self.published_targets={}
                            self.published_targets[joint]=getattr(self,'last_targets',{}).get(
                                joint,float(wire['data']['angle']))
                    if line.startswith('[feedback-timeout] '):
                        self.fault='All joint feedback recovery timeout'
                    if line.startswith(('[设备拒绝]','[保护停止]','[反馈中断]','[接收超时]','[未发送]','[失败]','[参数错误]',
                                        '[启动失败]','[启动未确认]','[未确认]','[执行/状态未确认]')):self.fault=self.fault or line
                    self.changed.notify_all()
        except Exception as error:
            with self.changed:self.fault=str(error);self.changed.notify_all()
        finally:
            with self.changed:
                if not self.closed:self.fault=self.fault or 'CLI exited'
                self.changed.notify_all()

    def _joint_feedback_event(self,event,stamp):
        """Consume the driver's per-axis decision; raw message remains evidence."""
        message=event['message']
        if message.get('address')!=2 or message.get('funcode')!=1:
            raise RuntimeError('Invalid per-joint feedback message')
        times=event['joint_times_s'];sequences=event['joint_sequences']
        pending=event['joint_pending'];merged=event['merged_angles']
        if any(not isinstance(x,list) or len(x)!=7 for x in (times,sequences,pending,merged)):
            raise RuntimeError('Invalid per-joint feedback metadata')
        accepted=event['accepted_joints'];rejected=event['rejected_joints']
        if (not isinstance(accepted,list) or not isinstance(rejected,list)
                or any(type(i) is not int or not 0<=i<7 for i in accepted+rejected)
                or sorted(accepted+rejected)!=list(range(7))):
            raise RuntimeError('Invalid per-joint feedback decisions')
        previous=list(getattr(self,'joint_q_time',[-math.inf]*7))
        previous_seq=list(getattr(self,'joint_q_seq',[0]*7))
        stamps=[]
        for joint,(value,seq,wait,q) in enumerate(zip(times,sequences,pending,merged)):
            value=-math.inf if value is None else value
            if (type(value) not in (int,float) or math.isnan(value) or value==math.inf
                    or (math.isfinite(value) and value<0)
                    or value>stamp+.00001 or value<previous[joint]
                    or type(seq) is not int or seq<previous_seq[joint] or type(wait) is not bool
                    or (q is not None and (type(q) not in (int,float) or not math.isfinite(q)))):
                raise RuntimeError('Invalid per-joint feedback state')
            if joint in accepted:
                if q is None or abs(value-stamp)>.00001 or seq<=previous_seq[joint] or wait:
                    raise RuntimeError('Invalid accepted joint feedback')
            elif (value!=previous[joint] or seq!=previous_seq[joint] or not wait
                    or (self.q is not None and q!=float(self.q[joint]))):
                raise RuntimeError('Rejected joint refreshed valid feedback')
            stamps.append(float(value))
        self.joint_q_time=stamps;self.joint_q_seq=list(sequences)
        self.joint_feedback_pending=list(pending);self.feedback_pending=any(pending)
        if 'all_feedback_gap_count' in event:
            count=event['all_feedback_gap_count']
            if type(count) is not int or count<getattr(self,'all_feedback_gap_count',0):
                raise RuntimeError('Invalid all-joint feedback gap history')
            start=event.get('last_all_feedback_gap_start_s');end=event.get('last_all_feedback_gap_end_s')
            if count and (type(start) not in (int,float) or type(end) not in (int,float)
                          or not math.isfinite(start) or not math.isfinite(end)
                          or not 0<=start<=end<=stamp+.00001):
                raise RuntimeError('Invalid all-joint feedback gap timing')
            self.feedback_gap_history_available=True;self.all_feedback_gap_count=count
            self.last_all_feedback_gap_start_s=start;self.last_all_feedback_gap_end_s=end
        self.feedback_rejected_count+=bool(rejected)
        if all(q is not None for q in merged):
            self.q=np.asarray(merged,dtype=float)
        if accepted:
            self.q_time=max(stamps);self.q_seq+=1

    def snapshot(self):
        with self.lock:
            result=dict(q=None if self.q is None else self.q.copy(),q_time=self.q_time,q_seq=self.q_seq,
                        status=dict(self.status),status_time=dict(self.status_time),fault=self.fault,
                        feedback_pending=getattr(self,'feedback_pending',False),
                        feedback_pause_total_s=getattr(self,'feedback_pause_total_s',0.),
                        feedback_rejected_count=getattr(self,'feedback_rejected_count',0),
                        joint_q_time=list(getattr(self,'joint_q_time',[self.q_time]*7)),
                        joint_q_seq=list(getattr(self,'joint_q_seq',[self.q_seq]*7)),
                        joint_feedback_pending=list(getattr(self,'joint_feedback_pending',
                                                           [getattr(self,'feedback_pending',False)]*7)),
                        raw_q=None if getattr(self,'raw_q',None) is None else self.raw_q.copy(),
                        raw_q_time=getattr(self,'raw_q_time',-math.inf))
            result['onboard_diagnostics']=dict(getattr(self,'onboard_diagnostics',{}))
            result['onboard_diagnostics_time']=getattr(self,'onboard_diagnostics_time',-math.inf)
            if getattr(self,'feedback_gap_history_available',False):
                result.update(all_feedback_gap_count=self.all_feedback_gap_count,
                    last_all_feedback_gap_start_s=self.last_all_feedback_gap_start_s,
                    last_all_feedback_gap_end_s=self.last_all_feedback_gap_end_s)
            return result

    def wait(self,seconds):
        with self.changed:self.changed.wait(timeout=max(0.,seconds))

    def send(self,q,check):
        """Send only changed arm targets; never reissue J6 or inactive holds.

        The board's all-joint path also publishes individual servo targets.
        Keeping that order here avoids sending identical targets to held axes.
        Three decimal places match the CLI wire precision, not jaw calibration.
        """
        return self._send(q,check)

    def retry_endpoint(self,q,check,tolerance_deg):
        """Reissue only still-outstanding arm targets, never a same-position hold."""
        if (isinstance(tolerance_deg,bool) or not math.isfinite(tolerance_deg)
                or not 0<tolerance_deg<=ARM_POSITION_TOLERANCE_DEG):
            raise ValueError('Endpoint retry requires a tolerance in (0, 1] degrees')
        return self._send(q,check,retry_tolerance_deg=tolerance_deg)

    def _send(self,q,check,*,retry_tolerance_deg=None):
        sent=0
        for joint in getattr(self,'active_joints',range(6)):
            if retry_tolerance_deg is not None and not 0<=joint<6:continue
            while True:
                admit=getattr(check,'admit_joint',None) if retry_tolerance_deg is None else None
                if callable(admit):admit(joint,float(q[joint]))
                checked_sequence=getattr(self,'q_seq',None)
                check()
                with self.lock:
                    # The reader and the actual write share this lock. A reject
                    # arriving after check() cannot let a new target slip through.
                    guard=dict(q=self.q,q_time=self.q_time,
                        joint_q_time=getattr(self,'joint_q_time',[self.q_time]*7),
                        feedback_pending=getattr(self,'feedback_pending',False),
                        joint_feedback_pending=getattr(self,'joint_feedback_pending',
                                                       [getattr(self,'feedback_pending',False)]*7))
                    if not all_feedback_fresh(guard,self.now()):
                        continue
                    if checked_sequence is not None and self.q_seq!=checked_sequence:
                        continue  # Recheck every guard against this newly arrived frame.
                    group_guard=getattr(check,'admit_group',None) if retry_tolerance_deg is None else None
                    if callable(group_guard):group_guard(q,self.q.copy(),joint)
                    value=round(float(q[joint]),3)
                    actual=float(self.q[joint])
                    error=abs(float(q[joint])-actual)
                    skipped=(error<=retry_tolerance_deg+1e-5 if retry_tolerance_deg is not None else
                             value==round(actual,3) or self.last_targets.get(joint)==value)
                    if skipped:break
                    if callable(admit) and error>FIXED_PROGRESS_MARGIN_DEG+1e-5:
                        continue  # The next loop waits outside the reader's lock.
                    delay_ms=(1 if getattr(self,'position_delay_policy','archived_formula')=='sdk_minimum'
                              else max(1,int(200*abs(value-float(self.q[joint]))/3)))
                    before=self.acks
                    context=dict(getattr(self,'reference_context',{}))
                    ordinal=self.commands+1
                    retry_info=(dict(endpoint_retry=True,endpoint_reference_sdk_units=float(q[joint]),
                                     measured_before_sdk_units=actual,endpoint_error_deg=error)
                                if retry_tolerance_deg is not None else {})
                    self.log.write(json.dumps(dict(t=self.now(),kind='command',joint=joint,target_sdk_units=value,
                        command_ordinal=ordinal,requested_delay_ms=delay_ms,
                        time_semantics='host CLI stdin submission; not device application',
                        **context,**retry_info))+'\n');self.log.flush()
                    self.commands+=1
                    self.proc.stdin.write(f'j {joint} {value:.3f} {delay_ms}\n');self.proc.stdin.flush()
                    self.last_targets[joint]=value
                    break
            if skipped:continue
            until=self.now()+.6
            pause_seen=getattr(self,'feedback_pause_total_s',0.)
            recovery_seen=getattr(self,'feedback_recovery_extra_pause_s',0.)
            while True:
                check()
                with self.lock:
                    pause=getattr(self,'feedback_pause_total_s',0.)
                    until+=pause-pause_seen;pause_seen=pause
                    recovery=getattr(self,'feedback_recovery_extra_pause_s',0.)
                    until+=recovery-recovery_seen;recovery_seen=recovery
                    if self.acks>before:break
                if self.now()>until:raise TimeoutError('Receipt timeout; no next command')
                self.wait(.01)
            sent+=1
            if context:
                with self.lock:
                    self.log.write(json.dumps(dict(t=self.now(),kind='command_ack',joint=joint,
                        command_ordinal=ordinal,target_sdk_units=value,
                        time_semantics='host observed CLI receipt completion; not arrival',**context))+'\n')
                    self.log.flush()
        return sent

    def close(self):
        with self.lock:self.closed=True
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        self.reader.join(timeout=2);self.proc.stdin.close();self.proc.stdout.close();self.log.close()


def healthy_status(s,now,*,allow_unstarted_position=False):
    if s['fault']:raise RuntimeError(s['fault'])
    for key,required in [('power_status',(1,)),('enable_status',(0,1) if allow_unstarted_position else (1,)),('error_status',(0,))]:
        if type(s['status'].get(key)) is not int or s['status'].get(key) not in required or not 0<=now-s['status_time'].get(key,-math.inf)<=FEEDBACK_FRESHNESS_S:
            raise RuntimeError('Invalid/stale '+key)


def joint_feedback_times(s):
    values=s.get('joint_q_time',[s.get('q_time',-math.inf)]*7)
    if len(values)!=7:raise RuntimeError('Invalid joint feedback timestamps')
    result=np.array([-math.inf if v is None else v for v in values],dtype=float)
    if np.isnan(result).any() or np.isposinf(result).any():
        raise RuntimeError('Invalid joint feedback timestamps')
    return result


def joint_pending_flags(s):
    values=s.get('joint_feedback_pending',[s.get('feedback_pending',False)]*7)
    if len(values)!=7 or any(type(v) not in (bool,np.bool_) for v in values):
        raise RuntimeError('Invalid joint feedback pending flags')
    return np.asarray(values,dtype=bool)


def joint_feedback_sequences(s):
    values=s.get('joint_q_seq',[s.get('q_seq',0)]*7)
    if len(values)!=7:raise RuntimeError('Invalid joint feedback sequences')
    return np.asarray(values,dtype=np.int64)


def all_feedback_fresh(s,now):
    ages=now-joint_feedback_times(s)
    return (s.get('q') is not None and bool(np.all((ages>=0)&(ages<=FEEDBACK_FRESHNESS_S)))
            and not bool(np.any(joint_pending_flags(s))))


def healthy(s,now,*,allow_unstarted_position=False):
    if s['fault']:raise RuntimeError(s['fault'])
    ages=now-joint_feedback_times(s)
    if s['q'] is None or not np.all((ages>=0)&(ages<=FEEDBACK_FRESHNESS_S)):
        raise RuntimeError('Angle feedback stale')
    errors=feedback_limit_errors(s['q'])
    if errors:raise RuntimeError('Actual joint outside limits: '+'; '.join(errors))
    healthy_status(s,now,allow_unstarted_position=allow_unstarted_position)
    return s['q']


class FeedbackRecovery:
    """Pause command production during bounded gaps, without changing sample age."""
    def __init__(self,transport,cancel=lambda:False,external_check=lambda:None,
                 allow_unstarted_position=lambda:False,required_joint_timeout_s=None):
        self.transport=transport;self.cancel=cancel;self.external_check=external_check
        self.allow_unstarted_position=allow_unstarted_position
        self.extra_pause_s=0.;self.gap_wait_s=0.;self.gap_recoveries=0
        self.joint_timeout_events=[];self.joint_stale=np.zeros(7,dtype=bool)
        self.gap_history_seen=None
        self.required_joint_timeout_s=required_joint_timeout_s

    def metrics(self):
        return dict(feedback_gap_recoveries=self.gap_recoveries,
                    feedback_gap_wait_s=self.gap_wait_s,
                    feedback_recovery_extra_pause_s=self.extra_pause_s,
                    feedback_joint_timeout_events=list(self.joint_timeout_events),
                    feedback_timeout_policy=('all_joints_stale' if self.required_joint_timeout_s is None else
                                             'required_joint_deadline'),
                    required_joint_timeout_s=self.required_joint_timeout_s,
                    feedback_gap_timeout_s=FEEDBACK_RECOVERY_TIMEOUT_S)

    def diagnostics(self,s,now):
        def age(stamp):
            value=now-stamp
            return float(value) if math.isfinite(value) else None
        return dict(**self.metrics(),feedback_pending_at_end=bool(s.get('feedback_pending',False)),
                    joint_feedback_age_s=[age(v) for v in joint_feedback_times(s)],
                    stale_feedback_joints=np.flatnonzero(now-joint_feedback_times(s)>FEEDBACK_FRESHNESS_S).tolist(),
                    feedback_rejected_count=s.get('feedback_rejected_count',0),
                    final_raw_feedback_age_s=age(s.get('raw_q_time',-math.inf)),
                    final_valid_feedback_age_s=age(s.get('q_time',-math.inf)),
                    all_feedback_gap_count=s.get('all_feedback_gap_count'),
                    last_all_feedback_gap_start_s=s.get('last_all_feedback_gap_start_s'),
                    last_all_feedback_gap_end_s=s.get('last_all_feedback_gap_end_s'),
                    feedback_failure_monotonic_s=now)

    def record_joint_transitions(self,s,now):
        ages=now-joint_feedback_times(s)
        stale=ages>FEEDBACK_FRESHNESS_S
        for joint in np.flatnonzero(stale!=self.joint_stale):
            event=dict(event='stale' if stale[joint] else 'recovered',joint=int(joint),t=now,
                       age_s=float(ages[joint]) if math.isfinite(ages[joint]) else None)
            self.joint_timeout_events.append(event)
            log=getattr(self.transport,'log',None)
            lock=getattr(self.transport,'lock',None)
            if log is not None:
                with lock if lock is not None else nullcontext():
                    log.write(json.dumps(dict(kind='joint_feedback_timeout',**event))+'\n');log.flush()
        self.joint_stale=stale
        return ages

    def read(self,snapshot=None):
        t=self.transport;s=t.snapshot() if snapshot is None else snapshot
        wait_started=None;pause_started=0.;deadline=None;stale_gap=False;failure=None
        try:
            while True:
                now=t.now()
                if self.cancel():raise Cancelled('Cancellation requested')
                self.external_check()
                healthy_status(s,now,allow_unstarted_position=self.allow_unstarted_position())
                errors=feedback_limit_errors(s.get('q'))
                if errors:raise RuntimeError('Actual joint outside limits: '+'; '.join(errors))
                ages=self.record_joint_transitions(s,now)
                if self.required_joint_timeout_s is not None and np.any(ages>=self.required_joint_timeout_s):
                    joints=np.flatnonzero(ages>=self.required_joint_timeout_s).tolist()
                    raise RuntimeError('Required joint feedback recovery timeout: '+','.join(f'J{i}' for i in joints))
                latest=float(np.max(joint_feedback_times(s)))
                pending=bool(np.any(joint_pending_flags(s)))
                if np.any(ages<0):raise RuntimeError('Angle feedback stale')
                # Source-side gap history survives late consumer scheduling.
                # Healthy axes arriving during the wait keep the whole link
                # alive; one genuinely >2s all-axis gap cannot be erased later.
                history=s.get('all_feedback_gap_count')
                if history is not None:
                    if type(history) is not int or history<0:
                        raise RuntimeError('Invalid all-joint feedback gap history')
                    if self.gap_history_seen is not None and history>self.gap_history_seen:
                        raise RuntimeError('All joint feedback recovery timeout')
                    self.gap_history_seen=history
                elif deadline is not None and latest>deadline:
                    raise RuntimeError('All joint feedback recovery timeout')
                if not pending and np.all(ages<=FEEDBACK_FRESHNESS_S):
                    if wait_started is not None and stale_gap:self.gap_recoveries+=1
                    return s['q'].copy(),s
                if wait_started is None:
                    wait_started=now;pause_started=float(s.get('feedback_pause_total_s',0.))
                deadline=latest+FEEDBACK_RECOVERY_TIMEOUT_S
                stale_gap=stale_gap or bool(np.any(ages>FEEDBACK_FRESHNESS_S))
                if np.all(ages>=FEEDBACK_RECOVERY_TIMEOUT_S):
                    raise RuntimeError('All joint feedback recovery timeout')
                t.wait(min(.01,deadline-now))
                s=t.snapshot()
        except BaseException as error:
            failure=error
            raise
        finally:
            if wait_started is not None:
                elapsed=max(0.,t.now()-wait_started)
                driver_pause=max(0.,float(s.get('feedback_pause_total_s',0.))-pause_started)
                extra=max(0.,elapsed-driver_pause)
                self.extra_pause_s+=extra
                t.feedback_recovery_extra_pause_s=getattr(t,'feedback_recovery_extra_pause_s',0.)+extra
                if stale_gap:self.gap_wait_s+=elapsed
            if failure is not None:failure.feedback_diagnostics=self.diagnostics(s,t.now())


def paced_references(plan, maximum_step_deg=1.5):
    """Subdivide approved straight reference intervals; preserve every endpoint."""
    previous_time=previous=None
    for stamp,target in references(plan):
        target=np.asarray(target,dtype=float)
        if previous is None:
            yield stamp,target
        else:
            count=max(1,math.ceil(float(np.max(abs(target-previous)))/maximum_step_deg-1e-10))
            for i in range(1,count+1):
                u=i/count
                yield previous_time+(stamp-previous_time)*u,previous+(target-previous)*u
        previous_time,previous=stamp,target


def execute(plan,transport,cancel=lambda:False,external_check=lambda:None,feedback=lambda q:None,
            gripper_target=None,allow_position_start=False,endpoint_tolerance_deg=1.,
            progress_policy='scheduled',endpoint_policy=None,
            scheduling_policy='ack_then_period_v1'):
    """Run one arm stage; final arrival is checked from new angle samples.

    Cancellation stops future commands. A same-position hold is deliberately
    not sent: the archived controller timing fix has not been deployed.
    Stopping command production does not certify a physical emergency stop.

    The opt-in ACK overlap clock includes receipt time in each source interval,
    reanchoring overdue references instead of replaying a backlog. Runtime
    pauses can stretch intervals nonuniformly; this does not certify measured
    velocity or acceleration limits. The caller limits it to fixed-pose stages.
    """
    if scheduling_policy=='timed_group_v1':
        from .sdk_timed_executor import execute_timed
        return execute_timed(plan,transport,cancel=cancel,external_check=external_check,
            feedback=feedback,gripper_target=gripper_target,
            endpoint_tolerance_deg=endpoint_tolerance_deg,endpoint_policy=endpoint_policy)
    if (isinstance(endpoint_tolerance_deg,bool) or not math.isfinite(endpoint_tolerance_deg)
            or not 0<endpoint_tolerance_deg<=1.):
        raise ValueError('Endpoint tolerance must be positive and at most 1 degree')
    if progress_policy not in ('scheduled','feedback_paced'):
        raise ValueError('Unknown trajectory progress policy')
    if scheduling_policy not in ('ack_then_period_v1','ack_overlap_v1','shared_progress_v1'):
        raise ValueError('Unknown command scheduling policy')
    if scheduling_policy in ('ack_overlap_v1','shared_progress_v1') and progress_policy!='feedback_paced':
        raise ValueError('Adaptive scheduling requires the reviewed fixed-pose progress policy')
    endpoint_policy=checked_endpoint_policy(endpoint_policy)
    if endpoint_policy is not None and progress_policy!='feedback_paced':
        raise ValueError('Endpoint retries require the reviewed fixed-pose progress policy')
    while True:
        initial=transport.snapshot()
        baseline=healthy(initial,transport.now(),allow_unstarted_position=allow_position_start and getattr(transport,'acks',0)==0).copy()
        external_check()
        if cancel():raise Cancelled('Cancellation requested')
        if not initial.get('feedback_pending',False):break
        transport.wait(.01)
    if np.max(abs(baseline[:6]-plan['sdk'][0]))>ARM_POSITION_TOLERANCE_DEG+1e-5:raise ValueError('Plan start differs from current feedback')
    grip=baseline[6] if gripper_target is None else float(gripper_target)
    if not math.isfinite(grip) or not LIMITS[6,0]<=grip<=LIMITS[6,1]:
        raise ValueError('Gripper target must be finite and within SDK limits [0,65]')
    if abs(grip-baseline[6])>3:
        raise ValueError('Gripper target differs from fresh feedback by >3 SDK units')
    held=[i for i in range(7) if i not in plan['active_joints']]
    transport.active_joints=tuple(plan['active_joints'])
    reference=baseline.copy();begin=transport.now();sent=0
    committed_reference=baseline.copy()
    pause_seen=float(initial.get('feedback_pause_total_s',0.));initial_pause=pause_seen
    recovery=FeedbackRecovery(transport,cancel=cancel,external_check=external_check,
        allow_unstarted_position=lambda:allow_position_start and getattr(transport,'acks',0)==0,
        required_joint_timeout_s=FEEDBACK_RECOVERY_TIMEOUT_S if scheduling_policy=='shared_progress_v1' else None)
    command_wait_pause_s=0.
    checked_snapshot=initial
    paced=progress_policy=='feedback_paced'
    pacing_wait_s=0.;pacing_waits=0;adaptive_steps=[]
    overlap_ack=scheduling_policy=='ack_overlap_v1'
    scheduling_overrun_s=0.;command_group_schedule=[]
    shared_data={}
    def tracking_reference():
        current=committed_reference.copy()
        published=getattr(transport,'published_targets',getattr(transport,'last_targets',None))
        if isinstance(published,dict):
            current=baseline.copy()
            for joint in plan['active_joints']:
                if joint in published:current[joint]=published[joint]
        return current
    def diagnostics(q,**extra):
        actual_reference=tracking_reference()
        errors=np.abs(q[:6]-actual_reference[:6]);joint=int(np.argmax(errors))
        return dict(progress_policy=progress_policy,scheduling_policy=scheduling_policy,joint=joint,
            reference_sdk=actual_reference.tolist(),actual_sdk=q.tolist(),
            tracking_reference_basis='last_published_joint_targets',
            tracking_reference='last_published_joint_targets',
            max_error_deg=float(errors[joint]),feedback_pacing_wait_s=pacing_wait_s,
            adaptive_reference_steps=list(adaptive_steps),**extra)
    def check(allow_cancel=True,snapshot=None):
        nonlocal begin,pause_seen,checked_snapshot
        q,s=recovery.read(snapshot)
        checked_snapshot=s
        pause=float(s.get('feedback_pause_total_s',0.))+recovery.extra_pause_s
        begin+=pause-pause_seen;pause_seen=pause
        if max(abs(q[i]-baseline[i]) for i in held)>.5+1e-5:raise RuntimeError('Uncommanded joint drift')
        if np.max(abs(q[:6]-tracking_reference()[:6]))>3:
            error=RuntimeError('Tracking error >3 degrees')
            error.execution_diagnostics=diagnostics(q)
            raise error
        if abs(q[6]-grip)>3:raise RuntimeError('Gripper tracking error >3 SDK units')
        feedback(q.copy())
        return q
    def await_candidate(candidate,joints,*,allow_partial=False):
        nonlocal begin,pacing_wait_s,pacing_waits
        wait_start=transport.now();pause_start=pause_seen
        first=check().copy();waiting=False
        while True:
            q=check()
            joint=max(joints,key=lambda i:abs(q[i]-candidate[i]))
            candidate_error=float(abs(q[joint]-candidate[joint]))
            if candidate_error<=FIXED_PROGRESS_MARGIN_DEG+1e-5:
                proposal=(1.,candidate[:6].copy());break
            if allow_partial:
                proposal=admissible_reference(reference[:6],candidate[:6],q[:6],
                                             margin_deg=FIXED_PROGRESS_MARGIN_DEG)
                if proposal is not None:break
            waiting=True
            active_wait=max(0.,transport.now()-wait_start-(pause_seen-pause_start))
            detail=dict(next_reference_sdk=candidate.tolist(),next_reference_joint=joint,
                next_reference_max_error_deg=candidate_error,
                progression_limit_deg=FIXED_PROGRESS_MARGIN_DEG)
            if active_wait>2.:
                moved=float(np.max(abs(q[:6]-first[:6])))
                blocking_change=float(abs(q[joint]-first[joint]))
                blocking_initial_error=float(abs(candidate[joint]-first[joint]))
                blocking_progress=blocking_initial_error-candidate_error
                error=TimeoutError('Accepted commands but no measured joint progress' if blocking_progress<.2 else
                    ('Fixed-pose feedback did not catch up' if paced else 'Trajectory feedback did not catch up'))
                error.execution_diagnostics=diagnostics(q,wait_s=active_wait,
                    measured_change_deg=moved,wait_start_sdk=first.tolist(),
                    blocking_joint=int(joint),blocking_joint_change_deg=blocking_change,
                    blocking_joint_initial_error_deg=blocking_initial_error,
                    blocking_joint_error_deg=candidate_error,
                    blocking_joint_error_progress_deg=blocking_progress,**detail)
                raise error
            if pacing_wait_s+active_wait>30.:
                error=TimeoutError('Fixed-pose feedback wait budget exceeded' if paced else
                                   'Trajectory feedback wait budget exceeded')
                error.execution_diagnostics=diagnostics(q,wait_s=active_wait,**detail)
                raise error
            transport.wait(.01)
        extra=max(0.,transport.now()-wait_start-(pause_seen-pause_start))
        begin+=extra;pacing_wait_s+=extra
        if waiting:pacing_waits+=1
        return proposal,q.copy()
    def admit_joint(joint,target):
        # Recheck admission at the actual axis, after earlier axes' ACKs.
        # Reissuing an unchanged endpoint uses its separately reviewed policy.
        if abs(tracking_reference()[joint]-target)<.0005:return
        candidate=reference.copy();candidate[joint]=target
        await_candidate(candidate,(joint,))
    if scheduling_policy!='shared_progress_v1':check.admit_joint=admit_joint
    endpoint_metrics=None
    try:
        if scheduling_policy=='shared_progress_v1':
            from .shared_progress import run_shared_progress
            def shared_commit(candidate,count):
                nonlocal committed_reference
                committed_reference=candidate.copy()
            shared_data=run_shared_progress(plan,transport,check,shared_commit,baseline,grip,LIMITS,
                                           pause_total=lambda:pause_seen)
            reference=shared_data.pop('reference');sent=shared_data.pop('sent')
            command_group_schedule=shared_data.pop('command_group_schedule')
            scheduling_overrun_s=shared_data.pop('scheduling_overrun_s')
        source_references=iter([] if scheduling_policy=='shared_progress_v1' else
                               paced_references(plan) if paced else references(plan))
        pending_reference=None;previous_elapsed=0.
        while True:
            if pending_reference is not None:
                elapsed,target=pending_reference;pending_reference=None
            else:
                try:elapsed,target=next(source_references)
                except StopIteration:break
            adaptive_step=None
            while True:
                check()
                due=begin+elapsed
                if transport.now()>=due:break
                transport.wait(min(.02,due-transport.now()))
            overdue=max(0.,transport.now()-due)
            group_overrun=0.
            if overlap_ack:
                # Preserve one reference at a time. An overlong ACK group moves
                # the clock to now; following references retain their intervals
                # instead of replaying an overdue queue. Feedback recovery has
                # already moved begin in check(), so it is not counted again.
                begin+=overdue;scheduling_overrun_s+=overdue;group_overrun=overdue
            elif overdue>.5/plan['rate']:
                if not paced:raise RuntimeError('Missed command schedule; no catch-up burst')
                extra=overdue
                if pacing_wait_s+extra>30.:
                    error=TimeoutError('Fixed-pose feedback wait budget exceeded')
                    error.execution_diagnostics=diagnostics(check(),wait_s=extra)
                    raise error
                begin+=extra;pacing_wait_s+=extra
            if elapsed>0:
                # Admit the next approved reference using fresh feedback after
                # its due time. Intermediate servo residuals need not satisfy
                # the final 1-degree arrival test. Keep 0.5 degrees of margin
                # inside the unchanged 3-degree tracking fault threshold.
                proposal,admitted_q=await_candidate(np.r_[target,grip],tuple(range(6)),allow_partial=paced)
                fraction,candidate=proposal
                if fraction<1.:
                    # Only split this unconsumed approved segment. Interpolate
                    # its time by the same fraction, preserving all corners.
                    pending_reference=(elapsed,np.asarray(target).copy())
                    source_elapsed=previous_elapsed+fraction*(elapsed-previous_elapsed)
                    adaptive_step=dict(source_elapsed_s=source_elapsed,pending_elapsed_s=elapsed,
                        segment_fraction=fraction,previous_reference_sdk=reference.tolist(),
                        next_approved_reference_sdk=np.r_[target,grip].tolist(),
                        reference_sdk=np.r_[candidate,grip].tolist(),actual_sdk=admitted_q.tolist(),
                        max_error_deg=float(np.max(abs(candidate-admitted_q[:6]))))
                    elapsed=source_elapsed;target=candidate
            reference=np.r_[target,grip]
            previous_elapsed=elapsed
            if elapsed==0:continue  # Initial state is measured, not a same-position command.
            group_started=transport.now()
            group_due=begin+elapsed
            reanchor=0.
            if overlap_ack:
                # Adaptive subdivision can shorten elapsed after admission at
                # the original point's due time. Anchor the consumed source
                # point at its actual send start to keep the pending interval.
                reanchor=max(0.,group_started-group_due)
                begin+=reanchor
            group_pause_start=pause_seen
            group_event=dict(source_elapsed_s=elapsed,
                scheduled_due_monotonic_s=group_due,group_started_monotonic_s=group_started,
                group_completed_monotonic_s=None,
                group_start_basis='transport_send_call',commands=None,completed=False,
                schedule_overrun_s=group_overrun,reference_reanchor_s=reanchor,
                feedback_pause_s=0.)
            command_group_schedule.append(group_event)
            try:
                count=transport.send(reference,check)
            finally:
                group_event['group_completed_monotonic_s']=transport.now()
                group_event['feedback_pause_s']=max(0.,pause_seen-group_pause_start)
            group_event.update(commands=count,completed=True)
            committed_reference=reference.copy()
            sent+=1 if count is None else count
            if adaptive_step is not None:
                adaptive_step['commands']=count
                adaptive_steps.append(adaptive_step)
                log=getattr(transport,'log',None)
                if log is not None:
                    lock=getattr(transport,'lock',None)
                    with lock if lock is not None else nullcontext():
                        log.write(json.dumps(dict(kind='adaptive_reference',t=transport.now(),**adaptive_step))+'\n')
                        log.flush()
            if not overlap_ack and (count is None or count>0):
                # A synchronous receipt/bootstrap wait must not consume the
                # next reference interval. Start that interval only once this
                # group has completed, without replaying overdue references.
                extra=max(0.,transport.now()-(begin+elapsed))
                begin+=extra;command_wait_pause_s+=extra
        # Three *new* feedback samples must agree with the endpoint and each
        # other. A fixed-pose policy may reissue this same approved endpoint
        # to outstanding axes; ordinary grasp stages retain passive settling.
        settle_snapshot=transport.snapshot()
        seen=settle_snapshot['q_seq'];seen_joint_sequences=joint_feedback_sequences(settle_snapshot)
        samples=[];settle_begin=transport.now();settle_pause=pause_seen
        retry_rounds=retry_commands=0;retry_events=[];last_retry_seq=None
        initial_error=best_error=progress_anchor=None;progress_at=0.
        next_retry=endpoint_policy['retry_interval_s'] if endpoint_policy else math.inf
        seek_budget=endpoint_policy['max_seek_s'] if endpoint_policy else 5.
        def seek_elapsed():return max(0.,transport.now()-settle_begin-(pause_seen-settle_pause))
        def seek_metrics():
            return dict(endpoint_retry_policy=endpoint_policy,endpoint_seek_elapsed_s=seek_elapsed(),
                endpoint_retry_rounds=retry_rounds,endpoint_retry_commands=retry_commands,
                endpoint_no_progress_s=max(0.,seek_elapsed()-progress_at),
                endpoint_best_error_deg=best_error,endpoint_initial_error_deg=initial_error,
                endpoint_retry_events=retry_events)
        endpoint_metrics=seek_metrics
        def record_retry(count,q,sequence,*,complete):
            nonlocal sent,retry_commands,retry_rounds
            sent+=count;retry_commands+=count
            if count:
                retry_rounds+=1
                retry_events.append(dict(elapsed_s=seek_elapsed(),commands=count,
                    reference_sdk=reference.tolist(),actual_before_sdk=q.tolist(),
                    max_error_before_deg=float(np.max(abs(q[:6]-reference[:6]))),
                    feedback_sequence=sequence,command_group_completed=complete))
        def convergence_error(code,q):
            error=TimeoutError(code)
            error.execution_diagnostics=diagnostics(q)
            error.endpoint_diagnostics=seek_metrics()
            return error
        def observe_seek(q):
            nonlocal initial_error,best_error,progress_anchor,progress_at
            error_deg=float(np.max(abs(q[:6]-reference[:6])))
            if initial_error is None:initial_error=best_error=progress_anchor=error_deg
            best_error=min(best_error,error_deg)
            if endpoint_policy and progress_anchor-best_error>=endpoint_policy['min_progress_deg']-1e-5:
                progress_anchor=best_error;progress_at=seek_elapsed()
            return error_deg
        def seek_bounds(q,error_deg):
            if seek_elapsed()>=seek_budget:
                raise convergence_error('Endpoint convergence time budget exceeded',q)
            if (error_deg>endpoint_tolerance_deg+1e-5
                    and seek_elapsed()-progress_at>=endpoint_policy['stall_timeout_s']):
                raise convergence_error('Endpoint convergence stalled',q)
        def seek_check():
            q=check()
            seek_bounds(q,observe_seek(q))
            return q
        while seek_elapsed()<seek_budget:
            s=transport.snapshot();q=check(snapshot=s)
            s=checked_snapshot
            error_deg=observe_seek(q)
            sequences=joint_feedback_sequences(s)
            if s['q_seq']!=seen and np.all(sequences>seen_joint_sequences):
                seen=s['q_seq']
                seen_joint_sequences=sequences
                if np.max(abs(q[:6]-reference[:6]))<=endpoint_tolerance_deg+1e-5:samples.append(q)
                else:samples=[]
                if len(samples)>=3 and np.max(np.ptp(np.array(samples[-3:])[:,:6],axis=0))<=.3+1e-5:
                    return dict(completed=True,commands=sent,actual_sdk=q.tolist(),
                                progress_policy=progress_policy,scheduling_policy=scheduling_policy,
                                scheduling_overrun_s=scheduling_overrun_s,
                                command_group_schedule=list(command_group_schedule),
                                feedback_pacing_wait_s=pacing_wait_s,
                                feedback_pacing_waits=pacing_waits,
                                adaptive_reference_steps=list(adaptive_steps),
                                progression_limit_deg=FIXED_PROGRESS_MARGIN_DEG,
                                endpoint_tolerance_deg=endpoint_tolerance_deg,
                                endpoint_error_deg=(q[:6]-reference[:6]).tolist(),gripper_target_changed=False,
                                feedback_pause_s=pause_seen-initial_pause,
                                command_wait_pause_s=command_wait_pause_s,
                                feedback_rejected_count=s.get('feedback_rejected_count',0),
                                gripper_target_sdk=float(grip),
                                gripper_target_source='measured_start' if gripper_target is None else 'explicit',
                                **shared_data,
                                **(seek_metrics() if endpoint_policy else {}),
                                **recovery.metrics())
            if endpoint_policy:
                seek_bounds(q,error_deg)
                if (error_deg>endpoint_tolerance_deg+1e-5 and seek_elapsed()>=next_retry
                        and s['q_seq']!=last_retry_seq):
                    before_commands=getattr(transport,'commands',None)
                    try:
                        count=transport.retry_endpoint(reference,seek_check,endpoint_tolerance_deg)
                    except BaseException:
                        after_commands=getattr(transport,'commands',None)
                        if (type(before_commands) is int and type(after_commands) is int
                                and after_commands>=before_commands):
                            record_retry(after_commands-before_commands,q,s['q_seq'],complete=False)
                        raise
                    if not isinstance(count,int) or isinstance(count,bool) or count<0:
                        raise RuntimeError('Endpoint retry transport did not report command count')
                    record_retry(count,q,s['q_seq'],complete=True)
                    last_retry_seq=checked_snapshot['q_seq']
                    next_retry=seek_elapsed()+endpoint_policy['retry_interval_s']
            transport.wait(.02)
        if endpoint_policy:
            raise convergence_error('Endpoint convergence time budget exceeded',check())
        raise TimeoutError('Endpoint position not reached; receipt was not completion')
    except BaseException as error:
        error.adaptive_reference_steps=list(adaptive_steps)
        if endpoint_policy and endpoint_metrics is not None:
            error.endpoint_diagnostics=endpoint_metrics()
        if not hasattr(error,'feedback_diagnostics'):
            error.feedback_diagnostics=recovery.diagnostics(checked_snapshot,transport.now())
        error.command_wait_pause_s = command_wait_pause_s
        error.scheduling_policy = scheduling_policy
        if not hasattr(error,'scheduling_overrun_s'):error.scheduling_overrun_s=scheduling_overrun_s
        if not hasattr(error,'command_group_schedule'):error.command_group_schedule=list(command_group_schedule)
        for key,value in shared_data.items():
            if not hasattr(error,key):setattr(error,key,value)
        error.hold_command_received = False
        error.physical_stop_verified = False
        error.future_commands_stopped = True
        raise
