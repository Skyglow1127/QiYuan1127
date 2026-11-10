import numpy as np
from aic_drive.types import EpisodeConfig
from aic_drive.scenarios import DrivingEnv, ForwardIDMVehicle

def test_traffic_has_stable_ids_and_reproducible_public_observations():
    a=DrivingEnv(EpisodeConfig(scenario='traffic',seed=123,duration=1))
    b=DrivingEnv(EpisodeConfig(scenario='traffic',seed=123,duration=1))
    a.reset();b.reset()
    assert len(a.target_states())==4
    for _ in range(10):
        a.step_acceleration(-2);b.step_acceleration(-2)
        assert a.target_states()==b.target_states()
        assert {v.track_id for v in a.target_states()}=={1,2,3,4}
    assert a.done and b.done
    a.close();b.close()

def test_idm_actor_reacts_to_an_actual_ego_leader_and_cannot_reverse():
    e=DrivingEnv(EpisodeConfig(scenario='traffic'));e.reset()
    actor=e.road.vehicles[1]
    e.road.vehicles=[e.vehicle,actor]
    actor.position=np.array([100.,0.]);actor.speed=20.;actor.target_lane_index=actor.lane_index
    actor.enable_lane_change=False
    e.vehicle.position=np.array([130.,0.]);e.vehicle.speed=22.
    actor.act();fast=actor.action['acceleration']
    e.vehicle.speed=5.;actor.act()
    assert actor.action['acceleration']<fast
    actor.speed=.01;actor.action={'acceleration':-6.,'steering':0.};x=actor.position[0]
    actor.step(.02)
    assert actor.speed==0 and actor.position[0]>=x
    e.close()
