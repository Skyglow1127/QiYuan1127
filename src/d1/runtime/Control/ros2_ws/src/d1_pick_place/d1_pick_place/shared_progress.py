"""One source-time phase for all axes, above the existing position/ACK API.

Source geometry is immutable. Analytic derivatives below describe the modeled
phase transition, not motor motion. ACK holds and feedback recovery can break
wall-time derivative continuity; no hardware buffering/synchrony is assumed.
"""
import bisect
import math
import numpy as np
from . import path_timing

MAX_PHASE_RATE=.8
MAX_PHASE_ACCELERATION=1.
SLOW_LAG_DEG=.5
STOP_LAG_DEG=2.
STALL_TIMEOUT_S=2.


class SourcePath:
    def __init__(self,plan,limits):
        self.plan=plan;self.q=np.asarray(plan['sdk'],dtype=float)
        if self.q.ndim!=2 or self.q.shape[1]!=6 or len(self.q)<2 or not np.isfinite(self.q).all():
            raise ValueError('Shared progress requires a finite six-axis source path')
        bounds=np.asarray(limits)[:6]
        if np.any(self.q<bounds[:,0]-1e-6) or np.any(self.q>bounds[:,1]+1e-6):
            raise ValueError('Shared source path exceeds joint limits')
        self.profile=plan.get('timing_profile','edge_quintic_v1')
        self.ends=[];self.velocity_bound=np.zeros(6);self.acceleration_bound=np.zeros(6)
        elapsed=0.
        if self.profile==path_timing.PROFILE:
            self.runs=plan['runs']
            if not self.runs:raise ValueError('Empty shared source runs')
            previous=self.q[0];covered=0
            for run in self.runs:
                duration=float(run['duration_s']);a=np.asarray(run['start']);b=np.asarray(run['end'])
                if duration<=0 or not math.isfinite(duration):raise ValueError('Invalid source run duration')
                start_index=run.get('start_index');end_index=run.get('end_index')
                if (type(start_index) is not int or type(end_index) is not int or
                        start_index!=covered or not start_index<end_index<len(self.q) or
                        not math.isfinite(float(run.get('start_time_s',math.nan))) or
                        abs(float(run['start_time_s'])-elapsed)>1e-8):
                    raise ValueError('Shared run metadata does not cover source path in order')
                if (a.shape!=(6,) or b.shape!=(6,) or not np.isfinite(a).all() or not np.isfinite(b).all()
                        or not np.allclose(a,previous,rtol=0,atol=1e-8)
                        or not np.allclose(a,self.q[start_index],rtol=0,atol=1e-8)
                        or not np.allclose(b,self.q[end_index],rtol=0,atol=1e-8)):
                    raise ValueError('Shared run geometry differs from immutable source path')
                distance=float(np.max(np.abs(b-a)))
                if not math.isfinite(float(run['distance'])) or abs(float(run['distance'])-distance)>1e-8:
                    raise ValueError('Shared run distance differs from source geometry')
                interior=self.q[start_index:end_index+1]
                if distance<1e-14:
                    if np.max(np.abs(interior-a))>1e-10 or run['shape']!='hold':
                        raise ValueError('Invalid shared dwell geometry')
                else:
                    axis=int(np.argmax(np.abs(b-a)));fractions=(interior[:,axis]-a[axis])/(b[axis]-a[axis])
                    if (np.max(np.abs(interior-(a+fractions[:,None]*(b-a))))>1e-10 or
                            np.any(np.diff(fractions)<=0)):
                        raise ValueError('Shared run would skip a source corner or reversal')
                if (np.any(a<bounds[:,0]-1e-6) or np.any(a>bounds[:,1]+1e-6)
                        or np.any(b<bounds[:,0]-1e-6) or np.any(b>bounds[:,1]+1e-6)):
                    raise ValueError('Shared source run exceeds joint limits')
                delta=np.abs(b-a)
                if run['shape']=='quintic':
                    v=1.875*delta/duration;acc=10/math.sqrt(3)*delta/duration**2
                elif run['shape']=='cosine_cruise':
                    if (not all(math.isfinite(float(run[k])) for k in ('peak_speed','ramp_s','cruise_s','stretch'))
                            or run['peak_speed']<=0 or run['ramp_s']<=0 or run['cruise_s']<0 or run['stretch']<=0
                            or abs(duration-(2*run['ramp_s']+run['cruise_s'])*run['stretch'])>1e-8
                            or abs(distance-run['peak_speed']*(run['ramp_s']+run['cruise_s']))>1e-8):
                        raise ValueError('Invalid shared cosine timing metadata')
                    direction=delta/float(run['distance'])
                    v=direction*run['peak_speed']/run['stretch']
                    acc=direction*run['peak_speed']*math.pi/(2*run['ramp_s']*run['stretch']**2)
                elif run['shape']=='hold':v=acc=np.zeros(6)
                else:raise ValueError('Unsupported shared source shape')
                self.velocity_bound=np.maximum(self.velocity_bound,v)
                self.acceleration_bound=np.maximum(self.acceleration_bound,acc)
                elapsed+=duration;self.ends.append(elapsed)
                previous=b;covered=end_index
            if covered!=len(self.q)-1 or not np.allclose(previous,self.q[-1],rtol=0,atol=1e-8):
                raise ValueError('Shared run endpoint differs from immutable source path')
        elif self.profile=='edge_quintic_v1':
            self.durations=np.asarray(plan['ticks'],float)/float(plan['rate'])
            if len(self.durations)!=len(self.q)-1 or np.any(self.durations<=0) or not np.isfinite(self.durations).all():
                raise ValueError('Invalid shared source ticks')
            for delta,duration in zip(np.abs(np.diff(self.q,axis=0)),self.durations):
                self.velocity_bound=np.maximum(self.velocity_bound,1.875*delta/duration)
                self.acceleration_bound=np.maximum(self.acceleration_bound,10/math.sqrt(3)*delta/duration**2)
                elapsed+=duration;self.ends.append(elapsed)
        else:raise ValueError('Unsupported shared source timing profile')
        self.duration=elapsed
        if abs(float(plan['duration_s'])-elapsed)>1e-6:raise ValueError('Shared source duration mismatch')
        self.speed_limit=float(plan.get('max_velocity_deg_s',max(self.velocity_bound)))
        self.acceleration_limit=float(plan.get('max_acceleration_deg_s2',max(self.acceleration_bound)))
        if (not math.isfinite(self.speed_limit) or not math.isfinite(self.acceleration_limit)
                or self.speed_limit<=0 or self.acceleration_limit<=0):
            raise ValueError('Invalid shared derivative limits')
        if max(self.velocity_bound)>self.speed_limit+1e-6 or max(self.acceleration_bound)>self.acceleration_limit+1e-6:
            raise ValueError('Source derivatives exceed reviewed bounds')

    def state(self,phase):
        phase=min(max(float(phase),0.),self.duration)
        index=min(bisect.bisect_left(self.ends,phase),len(self.ends)-1)
        start=0. if index==0 else self.ends[index-1]
        if self.profile==path_timing.PROFILE:
            return path_timing.run_state(self.runs[index],phase-start)
        duration=self.durations[index];u=min(1.,max(0.,(phase-start)/duration))
        delta=self.q[index+1]-self.q[index]
        return (self.q[index]+delta*(10*u**3-15*u**4+6*u**5),
                delta*30*u*u*(1-u)**2/duration,
                delta*60*u*(1-u)*(1-2*u)/duration**2)

    def boundary(self,phase):
        i=bisect.bisect_right(self.ends,phase+1e-10)
        return self.ends[min(i,len(self.ends)-1)]


class PhaseGovernor:
    """A smoothstep rate transition has bounded phase acceleration and zero
    phase acceleration at tick endpoints. The global chain-rule budget includes
    both source acceleration and the additional source-velocity * phase-accel.
    """
    def __init__(self,path,margin_deg=2.5):
        self.path=path;self.margin=margin_deg;self.phase=0.;self.rate=0.
        moving=path.velocity_bound>1e-12
        budget=(path.acceleration_limit-path.acceleration_bound*MAX_PHASE_RATE**2)
        allowed=budget[moving]/path.velocity_bound[moving] if np.any(moving) else np.array([1.])
        self.acceleration=min(MAX_PHASE_ACCELERATION,float(min(allowed)))
        if self.acceleration<=0:raise ValueError('No shared phase-acceleration budget')

    def propose(self,actual,dt):
        q,_,_=self.path.state(self.phase)
        errors=np.abs(q-np.asarray(actual)[:6]);joint=int(np.argmax(errors));lag=float(errors[joint])
        u=np.clip((STOP_LAG_DEG-lag)/(STOP_LAG_DEG-SLOW_LAG_DEG),0.,1.)
        desired=MAX_PHASE_RATE*float(u*u*(3-2*u))
        next_rate=float(np.clip(desired,self.rate-self.acceleration*dt/1.5,self.rate+self.acceleration*dt/1.5))
        next_rate=max(0.,min(MAX_PHASE_RATE,next_rate));delta_rate=next_rate-self.rate
        def phase_at(u):return self.phase+dt*(self.rate*u+delta_rate*(u**3-.5*u**4))
        fraction=1.;boundary=self.path.boundary(self.phase)
        if phase_at(1.)>boundary:
            low,high=0.,1.
            for _ in range(45):
                middle=(low+high)/2
                if phase_at(middle)<boundary:low=middle
                else:high=middle
            fraction=(low+high)/2
        phase=min(boundary,phase_at(fraction))
        if boundary-phase<1e-10:phase=boundary
        rate=self.rate+delta_rate*(3*fraction*fraction-2*fraction**3)
        phase_accel=6*delta_rate*fraction*(1-fraction)/dt
        target,source_v,source_a=self.path.state(phase)
        candidate_error=float(max(np.abs(target-np.asarray(actual)[:6])))
        peak_v=np.zeros(6);peak_a=np.zeros(6)
        for sample_u in np.linspace(0.,fraction,17):
            _,v,a=self.path.state(phase_at(sample_u))
            r=self.rate+delta_rate*(3*sample_u**2-2*sample_u**3)
            alpha=6*delta_rate*sample_u*(1-sample_u)/dt
            peak_v=np.maximum(peak_v,np.abs(v*r))
            peak_a=np.maximum(peak_a,np.abs(a*r*r+v*alpha))
        return dict(source_elapsed_s=phase,previous_source_elapsed_s=self.phase,
            phase_rate=rate,previous_phase_rate=self.rate,desired_phase_rate=desired,
            phase_acceleration=phase_accel,phase_acceleration_peak=1.5*abs(delta_rate)/dt,
            integration_dt_s=dt,transition_fraction=fraction,
            reference_sdk=target.tolist(),source_velocity_deg_s=source_v.tolist(),
            source_acceleration_deg_s2=source_a.tolist(),
            reference_velocity_deg_s=(source_v*rate).tolist(),
            reference_acceleration_deg_s2=(source_a*rate**2+source_v*phase_accel).tolist(),
            reference_velocity_peak_deg_s=peak_v.tolist(),
            reference_acceleration_peak_deg_s2=peak_a.tolist(),
            tracking_lag_deg=lag,binding_joint=joint,candidate_error_deg=candidate_error,
            boundary_reached=phase==boundary)

    def accept(self,step):
        self.phase=step['source_elapsed_s'];self.rate=step['phase_rate']


def run_shared_progress(plan,transport,check,commit,baseline,grip,limits,pause_total):
    """Complete source-path traversal; endpoint verification belongs to execute.
    No command bypasses health or margin checks. Mid-group safety loss aborts
    future commands; it cannot undo an earlier axis or physically stop the arm.
    """
    path=SourcePath(plan,limits);governor=PhaseGovernor(path)
    if not math.isfinite(plan['rate']) or not 0<plan['rate']<=10:
        raise ValueError('Shared reference tick rate must be in (0,10] Hz')
    period=1/float(plan['rate'])
    epoch=transport.now();transport.reference_epoch_monotonic_s=epoch
    events=[];groups=[];sent=0;overrun=0.;tick=0
    previous_tick=epoch;due=epoch+period;pause_seen=pause_total()
    progress_phase=0.;progress_at=epoch;progress_pause_anchor=pause_seen
    metrics=dict(policy='shared_progress_v1',max_phase_rate=MAX_PHASE_RATE,
        max_phase_acceleration_s_inv=governor.acceleration,
        source_velocity_bound_deg_s=path.velocity_bound.tolist(),
        source_acceleration_bound_deg_s2=path.acceleration_bound.tolist(),
        reference_velocity_bound_deg_s=(path.velocity_bound*MAX_PHASE_RATE).tolist(),
        reference_acceleration_bound_deg_s2=(path.acceleration_bound*MAX_PHASE_RATE**2+
                                           path.velocity_bound*governor.acceleration).tolist(),
        source_duration_s=path.duration,stall_timeout_s=STALL_TIMEOUT_S,
        phase_pause_resets=0,schedule_overrun_ticks=0,physical_derivatives_guaranteed=False,
        derivative_scope='analytic uninterrupted active-time transitions; ACK holds and recovery can interrupt wall-time continuity',
        source_geometry_preserved=True,hardware_atomic_group=False)
    transport.shared_progress_state=dict(source_elapsed_s=0.,phase_rate=0.,reference_id=0)
    def admit_group(candidate,actual,joint=None):
        errors=np.abs(np.asarray(candidate)[:6]-np.asarray(actual)[:6]);binding=int(np.argmax(errors))
        if max(errors)>2.5+1e-5:
            error=RuntimeError('Shared group margin lost; future axes stopped; physical atomicity not available')
            error.execution_diagnostics=dict(joint=binding,next_reference_joint=binding,
                next_reference_max_error_deg=float(errors[binding]),progression_limit_deg=2.5,
                next_reference_sdk=np.asarray(candidate).tolist(),actual_sdk=np.asarray(actual).tolist(),
                submitting_joint=joint,reference_id=tick)
            raise error
    check.admit_group=admit_group
    try:
        while governor.phase<path.duration-1e-10:
            actual=check()
            while transport.now()<due-1e-9:
                transport.wait(min(.02,due-transport.now()));actual=check()
            at=transport.now();pause=pause_total();freeze=max(0.,pause-pause_seen)
            if freeze>1e-8:
                governor.rate=0.;metrics['phase_pause_resets']+=1
            active_dt=min(period,max(0.,at-previous_tick-freeze))
            previous_tick=at;pause_seen=pause
            if active_dt<1e-8:
                due=at+period;continue
            late=max(0.,at-due);overrun+=late
            if late>1e-5:metrics['schedule_overrun_ticks']+=1
            step=governor.propose(actual,active_dt);tick+=1
            step.update(reference_id=tick,original_reference_epoch_monotonic_s=epoch,
                original_plan_start_sdk=list(plan['sdk'][0]),host_wall_elapsed_s=at-epoch,
                group_started_monotonic_s=at,group_completed_monotonic_s=None,
                scheduled_due_monotonic_s=due,schedule_overrun_s=late,
                feedback_freeze_s=freeze,completed=False,commands=None,
                commands_attempted=None,commands_acknowledged=None,hardware_atomic_group=False,
                group_start_basis='transport_send_call')
            events.append(step);groups.append(step)
            candidate=np.r_[step['reference_sdk'],grip]
            admit_group(candidate,actual)
            if step['source_elapsed_s']>governor.phase+1e-12:
                transport.shared_progress_state={k:step[k] for k in ('source_elapsed_s','phase_rate','reference_id')}
                transport.reference_context={k:step[k] for k in ('reference_id','source_elapsed_s',
                    'original_reference_epoch_monotonic_s','original_plan_start_sdk')}
                before_commands=getattr(transport,'commands',None);before_acks=getattr(transport,'acks',None)
                try:count=transport.send(candidate,check)
                finally:
                    after_commands=getattr(transport,'commands',None);after_acks=getattr(transport,'acks',None)
                    if type(before_commands) is int and type(after_commands) is int:
                        step['commands_attempted']=after_commands-before_commands
                    if type(before_acks) is int and type(after_acks) is int:
                        step['commands_acknowledged']=after_acks-before_acks
                    step['published_targets_at_group_end']=dict(getattr(transport,'published_targets',{}))
                    step['group_completed_monotonic_s']=transport.now()
                step['commands']=count;sent+=1 if count is None else count
                commit(candidate,count)
            else:
                step.update(commands=0,commands_attempted=0,commands_acknowledged=0)
            governor.accept(step);step['completed']=True
            step['group_completed_monotonic_s']=transport.now()
            watch_pause=pause_total()
            if governor.phase-progress_phase>=.01:
                progress_phase=governor.phase;progress_at=transport.now();progress_pause_anchor=watch_pause
            elif transport.now()-progress_at-max(0.,watch_pause-progress_pause_anchor)>STALL_TIMEOUT_S:
                error=TimeoutError('Shared progress stalled; no source-phase progress within 2 seconds')
                error.execution_diagnostics=dict(joint=step['binding_joint'],
                    source_elapsed_s=governor.phase,phase_rate=governor.rate,
                    max_error_deg=step['tracking_lag_deg'],actual_sdk=actual.tolist())
                raise error
            if transport.now()-epoch>4*path.duration+30:
                raise TimeoutError('Shared progress total execution budget exceeded')
            due=at+period
        metrics.update(source_elapsed_s=governor.phase,wall_elapsed_s=transport.now()-epoch,
                       references=tick,scheduling_overrun_s=overrun)
        return dict(reference=np.r_[path.state(path.duration)[0],grip],sent=sent,
            phase_events=events,command_group_schedule=groups,shared_progress_metrics=metrics,
            reference_epoch_monotonic_s=epoch,scheduling_overrun_s=overrun)
    except BaseException as error:
        if events and events[-1]['group_completed_monotonic_s'] is None:
            events[-1]['group_completed_monotonic_s']=transport.now()
        metrics.update(source_elapsed_s=governor.phase,wall_elapsed_s=transport.now()-epoch,references=tick)
        error.shared_progress_metrics=metrics;error.phase_events=events
        error.reference_epoch_monotonic_s=epoch;error.command_group_schedule=groups
        error.scheduling_overrun_s=overrun
        raise
    finally:
        if hasattr(check,'admit_group'):del check.admit_group
