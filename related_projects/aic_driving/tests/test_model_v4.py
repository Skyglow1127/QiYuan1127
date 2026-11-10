import numpy as np
import pytest
from aic_drive.types import TrackState,TrackSample
from aic_drive.model_v4 import features,SmoothedPredictor

def sample(t,v=10):return TrackSample(t,TrackState(1,10*t,0,v,0))
def test_causal_features_keep_missing_slots_and_real_timestamps():
    x=features((sample(.35),sample(.5)),np.array([.5,.8]))
    assert x.shape==(2,64)
    assert x[0,5]==1 and x[0,11]==1 and x[0,10]==pytest.approx(-.15)
    assert x[0,17]==0
    assert x[0,-4]==0 and x[1,-4]==pytest.approx(.1)
    with pytest.raises(ValueError):features((sample(.5),),[.4])
def test_smoothed_fit_uses_multiple_observations_and_stops_forward():
    h=tuple(TrackSample(i*.1,TrackState(1,10+i,0,10-2*i*.1,0)) for i in range(7))
    result=SmoothedPredictor(.6).predict(h,np.array([.6,1.6,9.6]))
    assert result[0,0]==h[-1].state.x
    assert result[1,2]==pytest.approx(6.8)
    assert result[-1,2]==0 and np.all(np.diff(result[:,0])>=0)

def test_features_exclude_stale_samples_and_reject_invalid_history():
    assert features((sample(0),sample(2)),[2.3])[0,11]==0
    with pytest.raises(ValueError):features((sample(float('nan')),),[1])
    with pytest.raises(ValueError):features((sample(.2),sample(.1)),[1])
    with pytest.raises(ValueError):features((sample(.1),TrackSample(.2,TrackState(2,2,0,10,0))),[1])

def test_checkpoint_online_gate_fallback_and_identity(tmp_path):
    from aic_drive.model_v4 import _torch,Net,OptimizedPredictor,SCHEMA
    torch=_torch();net=Net.build()
    with torch.no_grad():
        for p in net.parameters():p.zero_()
        net[-1].bias.fill_(1.)
    path=tmp_path/'model.pt'
    torch.save({'metadata':{'schema':SCHEMA,'window':.6,'supported_horizons':[.1,3.]},'states':[net.state_dict()],'mean':[0.]*64,'scale':[1.]*64},path)
    predictor=OptimizedPredictor(path);history=tuple(sample(i*.1) for i in range(7));queries=np.array([.6,1.6,4.6])
    base=SmoothedPredictor(.6).predict(history,queries);result=predictor.predict(history,queries)
    np.testing.assert_allclose(result[0],base[0]);np.testing.assert_allclose(result[1,:2]-base[1,:2],[.8,.8],atol=1e-6)
    np.testing.assert_allclose(result[2],base[2]);assert predictor.last_degraded
    np.testing.assert_allclose(predictor.predict((sample(.6),),[1.6]),SmoothedPredictor().predict((sample(.6),),[1.6]))
