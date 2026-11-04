import copy
from pathlib import Path
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, JointState
from geometry_msgs.msg import TransformStamped
from d1_pick_place.perception import Perception
from d1_pick_place.task import Task
from d1_pick_place.configuration import load_config
from d1_pick_place.offline import fixture
import json


class Recorder:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


@pytest.fixture
def node():
    rclpy.init()
    c = load_config(Path(__file__).parents[1] / 'config/task.yaml')
    c['handeye_validated'] = True
    node = Perception(c)
    node.targets, node.status = Recorder(), Recorder()
    tf = TransformStamped()
    tf.transform.rotation.w = 1.
    node.buffer.lookup_transform = lambda *args: tf
    yield node
    node.destroy_node()
    rclpy.shutdown()


def messages(node):
    color, depth, k, d = fixture()
    color = node.bridge.cv2_to_imgmsg(color, encoding='bgr8')
    depth = node.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
    info = CameraInfo(width=640, height=480, k=k.ravel().tolist(), d=d.tolist(), distortion_model='plumb_bob')
    for m in [color, depth, info]:
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = node.c['camera_frame']
    return color, depth, info


def test_ros_pose_pair_is_atomic_and_timestamped(node):
    rgb, depth, info = messages(node)
    node.process(rgb, depth, info)
    assert len(node.targets.messages) == 1
    msg = node.targets.messages[0]
    assert len(msg.poses) == 2
    assert msg.header.frame_id == 'base_link'
    assert msg.header.stamp == rgb.header.stamp
    assert json.loads(node.status.messages[-1].data)['ready']


@pytest.mark.parametrize('case', ['stale', 'future', 'skew', 'alignment', 'calibration', 'resolution', 'depth_encoding', 'handeye'])
def test_ros_rejects_invalid_inputs_without_publishing(node, case):
    rgb, depth, info = messages(node)
    if case == 'stale': rgb.header.stamp.sec -= 3
    if case == 'future': rgb.header.stamp.sec += 3
    if case == 'skew':
        depth.header.stamp.sec = rgb.header.stamp.sec
        depth.header.stamp.nanosec = max(0, rgb.header.stamp.nanosec - 100000000)
        if rgb.header.stamp.nanosec < 100000000: depth.header.stamp.sec -= 1
    if case == 'alignment': depth.header.frame_id = 'depth_optical'
    if case == 'calibration': info.distortion_model = 'equidistant'
    if case == 'resolution': info.width = 320
    if case == 'depth_encoding': depth.encoding = 'mono16'
    if case == 'handeye': node.c['handeye_validated'] = False
    node.process(rgb, depth, info)
    assert not node.targets.messages
    assert not json.loads(node.status.messages[-1].data)['ready']


def test_duplicate_feedback_does_not_count_as_new_frames():
    rclpy.init()
    c = load_config(Path(__file__).parents[1] / 'config/task.yaml')
    node = Task(c)
    try:
        msg = JointState(name=c['arm_joints'] + [c['gripper_joint']], position=[0.] * 7)
        msg.header.stamp = node.get_clock().now().to_msg()
        node.on_joints(msg)
        node.on_joints(copy.deepcopy(msg))
        assert len(node.history) == 1
        assert node.fresh_state()['Joint1'] == 0.
    finally:
        node.destroy_node()
        rclpy.shutdown()
