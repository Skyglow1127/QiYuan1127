import sys
import threading
from pathlib import Path
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import GoalResponse
from rclpy.serialization import serialize_message, deserialize_message
from d1_pick_place.sdk_executor import NAMES, Mapping
from d1_pick_place.hardware_bridge import Bridge, retime_trajectory, trajectory_plan, trajectory_digest


def trajectory():
    t = RobotTrajectory()
    t.joint_trajectory.joint_names = NAMES
    for q, sec in [(0., 0), (.02, 1)]:
        p = JointTrajectoryPoint(positions=[q, 0., 0., 0., 0., 0.])
        p.time_from_start.sec = sec
        t.joint_trajectory.points.append(p)
    return t


def test_retiming_is_idempotent_and_exposes_actual_duration():
    m = Mapping(np.ones(6), np.zeros(6))
    a = retime_trajectory(trajectory(), m)
    b = retime_trajectory(a, m)
    assert trajectory_digest(a) == trajectory_digest(b)
    p = a.joint_trajectory.points[-1].time_from_start
    assert p.sec + p.nanosec*1e-9 == trajectory_plan(a, m)['duration_s']
    assert p.sec > 1


def test_continuous_internal_waypoint_derivatives_survive_ros_roundtrip():
    t = trajectory()
    t.joint_trajectory.points.insert(1, JointTrajectoryPoint(positions=[.01, 0., 0., 0., 0., 0.]))
    t.joint_trajectory.points[1].time_from_start.nanosec = 500000000
    m = Mapping(np.array([-1., 1., 1., 1., 1., 1.]), np.zeros(6))
    a = retime_trajectory(t, m)
    b = retime_trajectory(a, m)
    assert trajectory_digest(a) == trajectory_digest(b)
    assert a.joint_trajectory.points[1].velocities[0] > 0
    assert list(a.joint_trajectory.points[0].velocities) == [0.]*6
    assert list(a.joint_trajectory.points[-1].velocities) == [0.]*6
    assert trajectory_plan(a, m)['continuous_internal_points'] == 1


def test_digest_ignores_cdr_padding_but_detects_changed_positions():
    t = trajectory()
    raw = bytearray(serialize_message(t))
    # Alignment after the first Joint1 name is padding, not a message field.
    raw[raw.index(b'Joint1\0') + len(b'Joint1\0')] = 165
    roundtrip = deserialize_message(bytes(raw), RobotTrajectory)
    assert trajectory_digest(roundtrip) == trajectory_digest(t)
    roundtrip.joint_trajectory.points[-1].positions[0] += .001
    assert trajectory_digest(roundtrip) != trajectory_digest(t)


def test_readonly_hash_mismatch_and_one_shot_gate():
    t = trajectory()
    snapshot = dict(q=np.zeros(7), q_time=0., q_seq=1, fault=None,
        status=dict(power_status=1, enable_status=1, error_status=0),
        status_time=dict(power_status=0., enable_status=0., error_status=0.))
    bridge = SimpleNamespace(lock=threading.Lock(), busy=False, used=False,
        args=SimpleNamespace(execute=False, trajectory_sha256=trajectory_digest(t)),
        mapping=Mapping(np.ones(6), np.zeros(6)),
        transport=SimpleNamespace(snapshot=lambda: snapshot, now=lambda: 0.),
        get_logger=lambda: SimpleNamespace(error=lambda _: None))
    goal = ExecuteTrajectory.Goal(trajectory=t)
    assert Bridge.accept_goal(bridge, goal) == GoalResponse.REJECT
    bridge.args.execute = True
    goal.trajectory.joint_trajectory.points[-1].positions[0] += .01
    assert Bridge.accept_goal(bridge, goal) == GoalResponse.REJECT
    goal.trajectory = t = trajectory()
    assert Bridge.accept_goal(bridge, goal) == GoalResponse.ACCEPT
    bridge.busy = False
    assert Bridge.accept_goal(bridge, goal) == GoalResponse.REJECT
