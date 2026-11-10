"""Execute one causal closed-loop episode and preserve raw evidence."""
from dataclasses import asdict
from collections import Counter
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
import hashlib
import importlib.metadata
import json
import platform
import os
import time
import numpy as np
from .types import EpisodeConfig, ControlDecision
from .scenarios import DrivingEnv
from .sensors import DelayedSensor, TrackHistory

ROOT=Path(__file__).resolve().parents[2]

def dump_json(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def source_manifest():
    files={}
    for base in [ROOT/'src',ROOT/'scripts',ROOT/'configs']:
        for p in sorted(base.rglob('*')):
            if p.is_file() and p.suffix in ('.py','.json'):
                files[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest()
    files['pyproject.toml']=hashlib.sha256((ROOT/'pyproject.toml').read_bytes()).hexdigest()
    for name in ['requirements.lock','requirements-torch.lock']:
        if (ROOT/name).exists():files[name]=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    combined=hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()
    packages={}
    for p in ['highway-env','gymnasium','numpy','torch','matplotlib']:
        try: packages[p]=importlib.metadata.version(p)
        except importlib.metadata.PackageNotFoundError: pass
    cpu=platform.processor()
    if Path('/proc/cpuinfo').exists():
        cpu=next((line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')),cpu)
    return {'source_sha256':combined,'files':files,'packages':packages,'python':platform.python_version(),
            'cpu_model':cpu,'logical_cpu_count':os.cpu_count(),'device':'cpu',
            'platform':platform.platform(),'created_at':datetime.now(ZoneInfo('Asia/Singapore')).isoformat()}

def make_controller(method,model_path=None,calibration_path=None,**overrides):
    from .prediction import AnalyticPredictor
    from .control import SafetyController
    if method not in ('cv_fixed','ca_fixed','ca_adaptive','ml_fixed','ml_adaptive','age_guard','ca_smooth','ml_smooth','ca_recovery','ml_recovery','fit_recovery','ml4_recovery','ml_retrained_recovery'):
        raise ValueError(f'Unknown method {method}')
    if method=='ml4_recovery':
        if not model_path:raise ValueError('ML methods require --model')
        from .model_v4 import OptimizedPredictor
        predictor=OptimizedPredictor(model_path,calibration_path)
    elif method=='fit_recovery':
        from .model_v4 import SmoothedPredictor
        predictor=SmoothedPredictor(calibration_path=calibration_path)
    elif method.startswith('ml'):
        if not model_path: raise ValueError('ML methods require --model')
        from .learning import LearnedPredictor
        predictor=LearnedPredictor(model_path,calibration_path)
    else:
        predictor=AnalyticPredictor(kind='cv' if method.startswith('cv') else 'ca')
        if calibration_path and not method.startswith('cv'):
            from .learning import CalibratedAnalyticPredictor
            predictor=CalibratedAnalyticPredictor(calibration_path,kind='ca')
    kwargs={'margin_mode':'adaptive' if method.endswith('adaptive') or method=='age_guard' or method.endswith(('smooth','recovery')) else 'fixed'}
    if method=='age_guard': kwargs['max_age']=0.45
    if method.endswith('smooth'): kwargs['smooth']=True
    if method.endswith('recovery'): kwargs.update(smooth=True,recovery_step=2.)
    kwargs.update(overrides)
    return SafetyController(predictor=predictor,**kwargs)

def controller_configuration(controller):
    """Record the actual instantiated controller, including injected overrides."""
    result={'class':f'{type(controller).__module__}.{type(controller).__qualname__}'}
    for name in ('target_speed','fixed_margin','max_age','guard','margin_mode','smooth','comfort_step','recovery_step',
                 'CONTROL_DT','HORIZON','EGO_BRAKING','LEAD_BRAKING','CANDIDATES'):
        if hasattr(controller,name):
            value=getattr(controller,name)
            result[name]=list(value) if isinstance(value,tuple) else value
    predictor=getattr(controller,'predictor',None)
    if predictor is not None:
        result['predictor']={'class':f'{type(predictor).__module__}.{type(predictor).__qualname__}'}
        for name in ('kind','uncertainty_kind','calibration_status','supported_horizons','window','schema'):
            if hasattr(predictor,name):result['predictor'][name]=getattr(predictor,name)
    return result

def interval_kinematics(times, speeds):
    """Finite-difference acceleration/jerk, including a partial terminal interval.

    Interval acceleration is located at its midpoint; jerk uses the difference
    between those midpoint times. This is an observed interval statistic, not
    an instantaneous actuator derivative at a command discontinuity.
    """
    t=np.asarray(times,dtype=float)
    v=np.asarray(speeds,dtype=float)
    if t.ndim!=1 or v.shape!=t.shape or len(t)<2 or not np.isfinite(t).all() or not np.isfinite(v).all():
        raise ValueError('At least two finite timestamped speed samples are required')
    dt=np.diff(t)
    if np.any(dt<=0):raise ValueError('Speed sample timestamps must strictly increase')
    acceleration=np.diff(v)/dt
    midpoint=(t[1:]+t[:-1])/2
    jerk_dt=np.diff(midpoint)
    jerk=np.diff(acceleration)/jerk_dt
    return dt,acceleration,jerk_dt,jerk

def run_episode(config:EpisodeConfig,method='ca_fixed',output_dir=None,controller=None,model_path=None,calibration_path=None):
    if output_dir is None:
        name=datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S_%f')
        output_dir=ROOT/'runs'/f'{name}_{config.scenario}_{method}'
    output_dir=Path(output_dir)
    output_dir.mkdir(parents=True,exist_ok=False)
    cfg=config.to_dict()
    cfg.update(method=method,model_path=str(model_path) if model_path else None,calibration_path=str(calibration_path) if calibration_path else None)
    dump_json(output_dir/'config.json',cfg)
    manifest=source_manifest()
    for name,path in [('model_sha256',model_path),('calibration_sha256',calibration_path)]:
        if path:manifest[name]=hashlib.sha256(Path(path).read_bytes()).hexdigest()
    dump_json(output_dir/'provenance.json',manifest)
    env=DrivingEnv(config)
    sensor=DelayedSensor(config.seed,config.delay,config.delay_mode,config.position_noise,config.velocity_noise,1/config.control_hz)
    history=TrackHistory()
    started=time.perf_counter()
    timings,prediction_timings,ages,speeds,speed_times=[],[],[],[],[]
    emergencies=degraded=emergency_events=0
    reasons,emergency_reasons=Counter(),Counter()
    diagnostics=Counter()
    was_emergency=False
    end_reason='exception'
    try:
        controller=controller or make_controller(method,model_path,calibration_path)
        cfg['effective_controller']=controller_configuration(controller)
        dump_json(output_dir/'config.json',cfg)
        if hasattr(controller,'CONTROL_DT') and not np.isclose(controller.CONTROL_DT,1/config.control_hz):
            raise ValueError('Controller action period differs from the environment control period')
        _,info=env.reset()
        dump_json(output_dir/'scenario.json',info['parameters'])
        with (output_dir/'trajectory.jsonl').open('w') as traj, (output_dir/'observations.jsonl').open('w') as obsfile:
            while not env.done:
                now=env.time
                truth=env.target_states()
                sensor.capture(now,truth)
                packets=sensor.receive(now)
                history.ingest(packets,now)
                ego=env.ego_state()
                start=time.perf_counter()
                decision=controller.decide(ego,history.snapshot())
                elapsed=(time.perf_counter()-start)*1000
                if not np.isfinite(decision.acceleration):raise ValueError('Non-finite control')
                cycle_diagnostics=dict(getattr(controller,'last_diagnostics',{}))
                diagnostics.update(cycle_diagnostics)
                timings.append(elapsed)
                prediction_timings.append(decision.prediction_ms)
                if decision.latest_observation_time is not None:
                    ages.append(float(decision.max_observation_age))
                emergencies+=int(decision.emergency)
                reasons[decision.reason]+=1
                if decision.emergency:emergency_reasons[decision.reason]+=1
                emergency_events+=int(decision.emergency and not was_emergency)
                was_emergency=decision.emergency
                degraded+=int(decision.degraded)
                speeds.append(ego.speed)
                speed_times.append(now)
                row={'time':now,'decision_at':now,'applied_at':now,'ego':asdict(ego),'targets':[asdict(x) for x in truth],
                     'decision':asdict(decision),'diagnostics':cycle_diagnostics,'crashed':False,'decision_ms':elapsed,'min_gap':minimum_gap(ego,truth)}
                traj.write(json.dumps(row,allow_nan=False)+'\n')
                obsfile.write(json.dumps({'time':now,'packets':[asdict(p) for p in packets]},allow_nan=False)+'\n')
                _,_,term,trunc,info=env.step_acceleration(decision.acceleration)
                if term or trunc:
                    end_reason='collision' if info['crashed'] else 'offroad' if info['offroad'] else 'time_limit'
                    end_ego=env.ego_state()
                    speeds.append(end_ego.speed)
                    speed_times.append(env.time)
                    traj.write(json.dumps({'time':env.time,'ego':asdict(end_ego),'targets':[asdict(x) for x in env.target_states()],
                          'decision':asdict(decision),'crashed':info['crashed'],'terminal':True,'min_gap':minimum_gap(end_ego,env.target_states())},allow_nan=False)+'\n')
            progress=float(env.vehicle.position[0]-env.initial_x)
            arr=np.asarray(timings)
            warmup=5 if len(arr)>5 else 0
            measured=arr[warmup:]
            cfg_hash=hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest()
            sample_dt,actual_accel,jerk_dt,jerk=interval_kinematics(speed_times,speeds)
            summary={'scenario':config.scenario,'seed':config.seed,'method':method,'delay':config.delay,'delay_mode':config.delay_mode,
                     'profile':config.profile,'position_noise':config.position_noise,'velocity_noise':config.velocity_noise,
                     'status':'completed','end_reason':end_reason,'collision':int(end_reason=='collision'),'offroad':int(end_reason=='offroad'),
                     'progress_m':progress,'progress_rate':progress/config.duration,'budget_s':config.duration,'duration_s':env.time,
                     'decision_p50_ms':float(np.median(measured)),'decision_p95_ms':float(np.percentile(measured,95)),
                     'decision_max_ms':float(arr.max()),'decision_over_100ms':int((arr>100).sum()),'timing_warmup_decisions':warmup,
                     'prediction_p95_ms':float(np.percentile(prediction_timings[warmup:],95)),
                     'prediction_timing_definition':'feature extraction, prediction and margins; excludes candidate evaluation; not neural-network-only latency',
                     'emergency_count':emergencies,'emergency_events':emergency_events,'degraded_count':degraded,
                     'candidate_diagnostics':dict(diagnostics),'reason_counts':dict(reasons),'emergency_reason_counts':dict(emergency_reasons),
                     'missing_observation_count':reasons['no_observation'],'stale_observation_count':reasons['stale_observation'],
                     'hard_brake_threshold_mps2':-4.,'hard_brake_tolerance_mps2':1e-6,
                     'hard_brake_cycles':int((actual_accel < -4-1e-6).sum()),
                     'hard_brake_duration_s':float(sample_dt[actual_accel < -4-1e-6].sum()),
                     'acceleration_interval_count':len(actual_accel),
                     'acceleration_interval_min_s':float(sample_dt.min()),'acceleration_interval_max_s':float(sample_dt.max()),
                     'jerk_rms':float(np.sqrt(np.average(jerk**2,weights=jerk_dt))) if len(jerk) else 0.,
                     'kinematics_metric_definition':'speed finite differences over actual intervals including terminal state; jerk at interval midpoints',
                     'max_age_s':max(ages,default=0.),'mean_age_s':float(np.mean(ages)) if ages else None,
                     'age_valid_cycles':len(ages),'age_mean_definition':'known observation cycles only; missing cycles counted separately',
                     'realized_packet_delay_mean_s':float(np.mean(sensor.delays)),
                     'discarded_out_of_order_tracks':history.out_of_order,
                     'wall_seconds':time.perf_counter()-started,'config_sha256':cfg_hash,
                     'source_sha256':manifest['source_sha256'],'run_dir':str(output_dir.resolve()),
                     'scope':'structured-state scripted simulation; not road validation'}
            dump_json(output_dir/'summary.json',summary)
            return summary
    except Exception as exc:
        dump_json(output_dir/'error.json',{'type':type(exc).__name__,'message':str(exc),'time':getattr(env,'time',None) if hasattr(env,'tick') else None})
        raise
    finally:env.close()

def minimum_gap(ego,targets):
    gaps=[s.x-ego.x-(s.length+ego.length)/2 for s in targets if s.x>=ego.x and abs(s.y-ego.y)<(s.width+ego.width)/2]
    return min(gaps) if gaps else None
