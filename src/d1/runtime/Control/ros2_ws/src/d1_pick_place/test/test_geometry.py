import copy
from pathlib import Path
import cv2
import numpy as np
import pytest
from d1_pick_place.geometry import detect, transform, validate_transform, from_pose, to_pose
from d1_pick_place.offline import fixture
from d1_pick_place.task import task_poses, validate_trajectory, scene_for
from d1_pick_place.configuration import load_config
from moveit_msgs.msg import RobotTrajectory, RobotState, AllowedCollisionMatrix, AllowedCollisionEntry
from trajectory_msgs.msg import JointTrajectoryPoint

CONFIG = Path(__file__).parents[1] / 'config/task.yaml'


def test_rgbd_cube_center_not_surface():
    cube, marker = detect(*fixture())
    local = np.linalg.inv(marker) @ cube
    assert local[2, 3] == pytest.approx(.0175, abs=.001)
    assert cube[2, 3] == pytest.approx(.5825, abs=.001)
    assert np.allclose(from_pose(to_pose(cube)), cube)


@pytest.mark.parametrize('case', ['red_on_table', 'bad_depth', 'wrong_marker_size', 'no_marker', 'two_cubes', 'wrong_cube_size'])
def test_reject_ambiguous_or_inconsistent_rgbd(case):
    color, depth, k, d = fixture()
    kwargs = {}
    if case == 'red_on_table': depth[:] = .6
    if case == 'bad_depth': depth[:] = np.nan
    if case == 'wrong_marker_size': kwargs['marker_size'] = .08
    if case == 'no_marker': color[210:271, 370:431] = 230
    if case == 'two_cubes':
        color[320:357, 242:279] = [0, 0, 255]
        depth[320:357, 242:279] = .565
    if case == 'wrong_cube_size':
        color[210:271, 230:291] = [0, 0, 255]
        depth[210:271, 230:291] = .565
    with pytest.raises(ValueError): detect(color, depth, k, d, **kwargs)


def test_hue_wraparound():
    color, depth, k, d = fixture()
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    hsv[222:259, 242:279, 0] = 179
    detect(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), depth, k, d)


@pytest.mark.parametrize('bad', [np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.full((4, 4), np.nan)])
def test_invalid_transform(bad):
    with pytest.raises(ValueError): validate_transform(bad)


def test_stage_heights_and_region():
    c = load_config(CONFIG)
    marker = transform(translation=[.2, 0, .05])
    cube = marker @ transform(translation=[-.1, 0, .0175])
    place, poses = task_poses(c, cube, marker)
    assert [p[0] for p in poses] == ['pregrasp', 'approach', 'lift', 'transfer', 'lower', 'retreat']
    assert place[2, 3] == pytest.approx(.0705)
    assert poses[0][1][2, 3] - poses[1][1][2, 3] == pytest.approx(.08)
    assert poses[1][1][2, 2] == -1
    cube[2, 3] += .03
    with pytest.raises(ValueError, match='same table'): task_poses(c, cube, marker)


def trajectory():
    t = RobotTrajectory()
    t.joint_trajectory.joint_names = ['Joint1', 'Joint5']
    for i in range(3):
        p = JointTrajectoryPoint(positions=[0., .01 * i])
        p.time_from_start.sec = i
        t.joint_trajectory.points.append(p)
    return t


def test_protected_trajectory_and_start():
    t = trajectory()
    initial = {'Joint1': 0., 'Joint5': 0.}
    assert validate_trajectory(t, initial, {'Joint1': 0.}, .0001, initial)['Joint5'] == .02
    t.joint_trajectory.points[1].positions[0] = .001
    with pytest.raises(ValueError, match='Protected'): validate_trajectory(t, initial, initial, .0001, initial)


@pytest.mark.parametrize('case', ['nan', 'time', 'jump', 'start', 'missing', 'floating'])
def test_bad_trajectories(case):
    t = trajectory()
    if case == 'nan': t.joint_trajectory.points[1].positions[1] = float('nan')
    if case == 'time': t.joint_trajectory.points[1].time_from_start.sec = 0
    if case == 'jump': t.joint_trajectory.points[1].positions[1] = .8
    if case == 'start': t.joint_trajectory.points[0].positions[1] = .1
    if case == 'missing': t.joint_trajectory.joint_names[1] = 'WrongJoint'
    if case == 'floating':
        from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
        t.multi_dof_joint_trajectory.points = [MultiDOFJointTrajectoryPoint()]
    with pytest.raises(ValueError): validate_trajectory(t, {'Joint1': 0., 'Joint5': 0.}, {}, .0001, ['Joint1', 'Joint5'])


def test_acm_preserves_existing_and_only_allows_finger_contact():
    acm = AllowedCollisionMatrix(entry_names=['base_link', 'Link1'],
                                 entry_values=[AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[True, False])])
    scene = scene_for(load_config(CONFIG), np.eye(4), np.eye(4), RobotState(), baseline_acm=acm)
    m = scene.allowed_collision_matrix
    assert m.entry_values[0].enabled[1]
    cube = m.entry_names.index('d1_task_cube')
    assert not m.entry_values[cube].enabled[0]
    for finger in ['Link7_1', 'Link7_2']:
        assert m.entry_values[cube].enabled[m.entry_names.index(finger)]
    assert len(acm.entry_names) == 2


def test_configuration_accepts_explicit_protection_subset(tmp_path):
    import yaml
    c = load_config(CONFIG)
    c['protected_joints'] = []
    p = tmp_path / 'config.yaml'
    p.write_text(yaml.safe_dump(c))
    assert load_config(p)['protected_joints'] == []
    for bad in [['Joint99'], ['Joint1', 'Joint1']]:
        c['protected_joints'] = bad
        p.write_text(yaml.safe_dump(c))
        with pytest.raises(ValueError, match='protected_joints'): load_config(p)


def test_first_lift_returns_to_same_place():
    c = load_config(CONFIG)
    c.update(task_mode='lift_return', cube_size_m=.04, lift_height_m=.03, place_offset_m=[0., 0.])
    cube = transform(translation=[0., 0., .02])
    place, poses = task_poses(c, cube, np.eye(4))
    assert [x[0] for x in poses] == ['pregrasp', 'approach', 'lift', 'lower', 'retreat']
    p = {n: t for n, t, _ in poses}
    assert np.allclose(p['lift'][:3, 3]-p['approach'][:3, 3], [0, 0, .03])
    assert np.allclose(place[:2, 3], cube[:2, 3])
    assert p['lower'][2, 3] == pytest.approx(.02+c['place_release_gap_m'])


def test_side_grasp_approach_follows_tool_axis_and_lift_is_vertical():
    c = load_config(CONFIG)
    c.update(task_mode='lift_return', cube_size_m=.04, lift_height_m=.03,
             place_offset_m=[0., 0.], grasp_style='side', side_pitch_deg=20.)
    _, poses = task_poses(c, transform(translation=[0.,0.,.02]), np.eye(4))
    p = {n:t for n,t,_ in poses}
    assert np.allclose(p['approach'][:3,3]-p['pregrasp'][:3,3], p['approach'][:3,2]*c['clearance_m'])
    assert np.allclose(p['lift'][:3,3]-p['approach'][:3,3], [0,0,.03])
    assert np.linalg.det(p['approach'][:3,:3]) == pytest.approx(1.)
