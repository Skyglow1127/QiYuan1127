"""Read-only sample quality for quantized feedback, with full raw evidence.

Not a movement controller. Limits, fresh ready status and gaps remain strict.
Use jointly with image stationarity, never as a substitute for it.
"""
import math
import numpy as np
from j4_smooth_test import JOINT_LIMITS


def assess_window(events, begin, end, final=True):
    minimum=6.0 if final else 1.5
    if not math.isfinite(begin+end) or end-begin<minimum:
        raise ValueError('Observation window too short')
    chosen=[e for e in events if begin<=e['t']<=end]
    if any(b['t']<a['t'] for a,b in zip(chosen,chosen[1:])):
        raise ValueError('Nonmonotonic observation timestamps')
    groups={k:[e for e in chosen if e['kind']==k] for k in ('angles','status')}
    gaps={}
    for kind,frames in groups.items():
        if len(frames)<3:raise ValueError('Insufficient '+kind+' feedback')
        stamps=[begin]+[e['t'] for e in frames]+[end]
        gaps[kind]=max(b-a for a,b in zip(stamps,stamps[1:]))
        if gaps[kind]>.6:raise ValueError(kind+' feedback gap >0.6s')
    if any(e['state']!=[1,1,0] for e in groups['status']):
        raise ValueError('Power/enable/error state changed or not ready')
    q=np.array([e['q'] for e in groups['angles']],float)
    t=np.array([e['t']-begin for e in groups['angles']])
    if q.ndim!=2 or q.shape[1]!=7 or not np.isfinite(q).all():
        raise ValueError('Invalid angle array')
    for j,(lo,hi) in enumerate(JOINT_LIMITS):
        if np.any((q[:,j]<lo)|(q[:,j]>hi)):
            raise ValueError(f'J{j} feedback outside existing limits')
    spans=np.ptp(q,axis=0)
    deviation=np.max(abs(q-np.median(q,axis=0)),axis=0)
    if np.any(deviation>.3+1e-5):
        raise ValueError('Robot not stationary: feedback outlier >0.3 from median')
    central=np.percentile(q,95,axis=0)-np.percentile(q,5,axis=0)
    bins=[]
    for i in range(int(end-begin)):
        values=q[(t>=i)&(t<i+1)]
        if len(values)<5:raise ValueError('Insufficient samples in a one-second block')
        bins.append(np.median(values,axis=0))
    median_spans=np.ptp(bins,axis=0)
    slopes=np.polyfit(t,q,1)[0]
    if final:
        if np.any(central[:6]>.2+1e-5) or central[6]>.3+1e-5:
            raise ValueError('Robot not stationary: central feedback spread too large')
        if np.any(median_spans[:6]>.1+1e-5) or median_spans[6]>.2+1e-5:
            raise ValueError('Robot not stationary: sustained block-median change')
        if np.any(np.abs(slopes[:6])>.02+1e-5) or abs(slopes[6])>.03+1e-5:
            raise ValueError('Robot not stationary: sustained feedback trend')
    return dict(begin_aipc_monotonic_s=begin,end_aipc_monotonic_s=end,
        sampled_stationarity_verified=bool(final),stationarity_method='quantized-feedback-plus-vision-v1',
        max_feedback_gaps_s=gaps,frame_counts={k:len(v) for k,v in groups.items()},
        joint_median=np.median(q,axis=0).tolist(),joint_spans=spans.tolist(),
        joint_p05_p95_spans=central.tolist(),one_second_median_spans=median_spans.tolist(),
        linear_trend_deg_s=slopes.tolist(),median_raw_deviation=deviation.tolist(),
        raw_deviation_from_median_limit=.3,central_arm_span_limit_deg=.2,block_median_arm_span_limit_deg=.1,
        arm_trend_limit_deg_s=.02,
        note='Requires independent image stationarity check; quantiles are noise diagnostics, not mechanical confidence intervals.')
