"""Causal richer-history residual model and independently jointly calibrated rectangles.
No collision or trajectory-coverage guarantee is implied by empirical point coverage.
"""
from pathlib import Path
import json
import numpy as np
from .prediction import AnalyticPredictor,longitudinal_motion
from .learning import _torch,_sha256,_residuals

SCHEMA='position-velocity-time-64-v1'
WINDOWS=(.4,.6,1.)
EDGES=np.array([0.,.5,1.,1.5,2.,2.5,3.2])

def features(samples,queries):
    if not samples:raise ValueError('Empty history')
    times=np.array([s.captured_at for s in samples]);q=np.asarray(queries,dtype=float)
    if not np.isfinite(times).all() or np.any(np.diff(times)<=0) or any(s.state.track_id!=samples[-1].state.track_id for s in samples):raise ValueError('Invalid history')
    if q.ndim!=1 or not np.isfinite(q).all() or np.any(q<times[-1]-1e-9):raise ValueError('Invalid future query')
    last=samples[-1].state;t=times[-1];first=np.searchsorted(times,t-1.-1e-9);samples=samples[first:];times=times[first:];base=np.zeros(64,dtype=np.float32)
    for j in range(10):
        i=np.searchsorted(times,t-j*.1+1e-9,side='right')-1
        if i>=0:
            st=samples[i].state
            base[6*j:6*j+6]=[(st.x-last.x)/30,(st.y-last.y)/5,(st.vx-last.vx)/10,(st.vy-last.vy)/5,(times[i]-t),1]
    base[-3:]=[last.vx/35,last.vy/5,last.y/4]
    result=np.tile(base,(len(q),1));result[:,-4]=(q-t)/3
    if not np.isfinite(result).all():raise ValueError('Nonfinite inputs')
    return result

class SmoothedPredictor:
    kind='least_squares_ca'
    uncertainty_kind='empirical_joint_rectangle'
    def __init__(self,window=.6,calibration_path=None):
        self.calibration=json.loads(Path(calibration_path).read_text()) if calibration_path else None
        self.window=self.calibration['window'] if self.calibration else window
        self.calibration_status='empirically_calibrated' if self.calibration else 'uncalibrated_fallback'
        self.last_degraded=False;self.last_uncertainty_degraded=False
    def predict(self,samples,queries):
        # Reuse the checked identity/time contract, then fit only past velocities.
        result=AnalyticPredictor('ca').predict(samples,queries)
        t=samples[-1].captured_at;recent=[s for s in samples if s.captured_at>=t-self.window-1e-9]
        self.last_degraded=len(recent)<3 or recent[-1].captured_at-recent[0].captured_at<.2-1e-8
        if self.last_degraded:return result
        tau=np.array([s.captured_at-t for s in recent]);design=np.column_stack([np.ones(len(tau)),tau])
        coeff=np.linalg.lstsq(design,np.array([[s.state.vx,s.state.vy] for s in recent]),rcond=None)[0]
        vx,vy=coeff[0];ax,ay=np.clip(coeff[1],[-8,-2],[3,2]);h=np.maximum(0,np.asarray(queries)-t);st=samples[-1].state
        x,vel=longitudinal_motion(st.x,max(0,float(vx)),float(ax),h)
        th=np.minimum(h,-vy/ay) if vy*ay<0 else h
        result=np.column_stack([x,st.y+vy*th+.5*ay*th**2,vel,vy+ay*th])
        return result
    def uncertainty(self,horizons):
        return margins(self.calibration,'fit',horizons,self)

class Net:
    @staticmethod
    def build():
        torch=_torch()
        return torch.nn.Sequential(torch.nn.Linear(64,96),torch.nn.ReLU(),torch.nn.Linear(96,96),torch.nn.ReLU(),torch.nn.Linear(96,2))

def predict_residual(models,x,mean,scale):
    gate=(x[:,-4]*3)**2;gate=gate/(gate+.25)
    return np.mean([_residuals(m,x,mean,scale) for m in models],axis=0)*gate[:,None]

def margins(cal,kind,h,owner):
    h=np.asarray(h,dtype=float)
    if h.ndim!=1 or not np.isfinite(h).all() or np.any(h<0):raise ValueError('Invalid horizon')
    if cal is None:
        owner.last_uncertainty_degraded=True
        return np.column_stack([.5+.8*h+.15*h*h,.3+.3*h])
    edges=np.array(cal['edges']);indices=np.clip(np.searchsorted(edges[1:],h,side='left'),0,len(edges)-2)
    values=np.asarray(cal[kind]['rectangles'])[indices].copy();outside=h>edges[-1]
    excess=np.maximum(h-edges[-1],0);values+=np.column_stack([.8*excess+.15*excess**2,.3*excess])
    if owner.last_degraded:
        values=np.maximum(values,np.column_stack([.5+.8*h+.15*h*h,.3+.3*h]))
    owner.last_uncertainty_degraded=bool(outside.any() or owner.last_degraded)
    return values

class OptimizedPredictor(SmoothedPredictor):
    kind='causal_residual_v4'
    def __init__(self,model_path,calibration_path=None):
        super().__init__(calibration_path=calibration_path)
        torch=_torch();checkpoint=torch.load(model_path,map_location='cpu',weights_only=True)
        self.metadata=checkpoint['metadata']
        if self.metadata['schema']!=SCHEMA:raise ValueError('Unsupported feature schema')
        self.schema=SCHEMA
        if self.calibration and self.calibration['window']!=self.metadata['window']:raise ValueError('Calibration/window mismatch')
        self.window=self.metadata['window'];self.mean=np.array(checkpoint['mean'],dtype=np.float32);self.scale=np.array(checkpoint['scale'],dtype=np.float32)
        self.models=[]
        for state in checkpoint['states']:
            model=Net.build();model.load_state_dict(state);model.eval();self.models.append(model)
        self.supported_horizons=self.metadata['supported_horizons']
        if self.calibration and self.calibration['model_sha256']!=_sha256(Path(model_path)):raise ValueError('Calibration/model mismatch')
    def predict(self,samples,queries):
        result=super().predict(samples,queries);h=np.asarray(queries)-samples[-1].captured_at
        low,high=self.supported_horizons;valid=(h>=low-1e-6)&(h<=high+1e-6)&(h>1e-9)
        if self.last_degraded:valid[:]=False
        if valid.any():result[valid,:2]+=predict_residual(self.models,features(samples,np.asarray(queries)[valid]),self.mean,self.scale)
        self.last_degraded=bool(self.last_degraded or np.any((~valid)&(h>1e-9)))
        return result
    def uncertainty(self,horizons):
        # Mixed unsupported queries use a conservative union of neural and fit tables.
        neural=margins(self.calibration,'nn',horizons,self)
        if self.last_degraded and self.calibration:neural=np.maximum(neural,margins(self.calibration,'fit',horizons,self))
        return neural
