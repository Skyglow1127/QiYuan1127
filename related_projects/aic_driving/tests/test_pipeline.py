from dataclasses import replace
import numpy as np
import pytest
from aic_drive.types import EpisodeConfig,TrackState,DetectionPacket
from aic_drive.scenarios import DrivingEnv
from aic_drive.sensors import DelayedSensor,TrackHistory
from aic_drive.runner import run_episode

def test_delay_and_snapshot():
    sensor=DelayedSensor(delay=.2)
    state=TrackState(7,100,0,20,0)
    sensor.capture(0.,[state])
    assert not sensor.receive(.199)
    p=sensor.receive(.2)[0]
    assert p.captured_at==0 and p.received_at==.2 and p.tracks[0].x==100
    history=TrackHistory();history.ingest([p],.2)
    assert history.snapshot()[7][-1].state.x==100
    with pytest.raises(ValueError):TrackHistory().ingest([p],.1)

def test_tracks_association_and_out_of_order():
    h=TrackHistory()
    s1=TrackState(1,100,0,20,0);s2=TrackState(2,200,4,20,0)
    h.ingest([DetectionPacket(0,0,(s1,s2))],0)
    h.ingest([DetectionPacket(.2,.2,(replace(s2,x=204),replace(s1,x=104)))],.2)
    h.ingest([DetectionPacket(.1,.3,(replace(s1,x=102),))],.3)
    assert h.snapshot()[1][-1].state.x==104
    assert h.snapshot()[2][-1].state.x==204
    assert h.out_of_order==1

def test_bursty_mean_and_noise_stream_independence():
    state=TrackState(1,100,0,20,0)
    fixed=DelayedSensor(seed=2,delay=.2,position_noise=.1)
    burst=DelayedSensor(seed=2,delay=.2,mode='bursty',position_noise=.1)
    for i in range(20):fixed.capture(i*.1,[state]);burst.capture(i*.1,[state])
    assert np.mean(burst.delays)==pytest.approx(.2)
    pf=sorted([x[2] for x in fixed.queue],key=lambda p:p.captured_at)
    pb=sorted([x[2] for x in burst.queue],key=lambda p:p.captured_at)
    assert [p.tracks[0].x for p in pf]==[p.tracks[0].x for p in pb]

@pytest.mark.parametrize('name',['follow','brake','cutin'])
def test_scenario_determinism(name):
    cfg=EpisodeConfig(scenario=name,seed=21,duration=1)
    a=DrivingEnv(cfg);b=DrivingEnv(cfg)
    a.reset();b.reset()
    assert a.parameters==b.parameters
    for _ in range(10):
        a.step_acceleration(-2);b.step_acceleration(-2)
    assert a.time==1 and a.done
    assert a.ego_state()==b.ego_state() and a.target_states()==b.target_states()
    with pytest.raises(RuntimeError):a.step_acceleration(0)

def test_action_mapping_and_stop():
    e=DrivingEnv(EpisodeConfig(duration=1));e.reset();e.vehicle.speed=.1
    assert e.action_type.get_action(np.array([.5]))['acceleration']==0
    for _ in range(10):e.step_acceleration(-6)
    assert e.vehicle.speed==0
    assert e.vehicle.position[0]>=100

def test_closed_loop_logs_and_denominator(tmp_path):
    result=run_episode(EpisodeConfig(scenario='follow',seed=21,duration=2,delay=.2),output_dir=tmp_path/'run')
    assert result['status']=='completed' and result['collision']==0
    assert result['progress_rate']==pytest.approx(result['progress_m']/2)
    assert (tmp_path/'run'/'observations.jsonl').exists()
    assert result['max_age_s']==pytest.approx(.2)
