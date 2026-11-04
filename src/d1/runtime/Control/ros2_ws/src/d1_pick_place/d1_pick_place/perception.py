"""ROS RGB-D adapter. Publishes cube + marker in ONE timestamped PoseArray."""
import argparse
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, TransformException
from .configuration import load_config, default_config
from .geometry import detect, from_pose, to_pose


class Perception(Node):
    def __init__(self, config):
        super().__init__('d1_red_cube_perception')
        self.c = config
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.targets = self.create_publisher(PoseArray, config['observations_topic'], 10)
        self.status = self.create_publisher(String, '/d1/perception/status', 10)
        self.subs = [Subscriber(self, kind, config[key], qos_profile=qos_profile_sensor_data)
                     for kind, key in [(Image, 'color_topic'), (Image, 'depth_topic'), (CameraInfo, 'info_topic')]]
        self.sync = ApproximateTimeSynchronizer(self.subs, 8, config['max_sync_skew_s'])
        self.sync.registerCallback(self.process)
        self.get_logger().info('Waiting for timestamped, aligned RGB-D and calibrated wrist TF')

    def process(self, color_msg, depth_msg, info):
        try:
            c = self.c
            stamps = [Time.from_msg(m.header.stamp).nanoseconds * 1e-9 for m in [color_msg, depth_msg, info]]
            now = self.get_clock().now().nanoseconds * 1e-9
            if any(t <= 0 or not 0 <= now - t <= c['max_frame_age_s'] for t in stamps):
                raise ValueError('Stale/future/zero RGB-D timestamp')
            if max(stamps) - min(stamps) > c['max_sync_skew_s']:
                raise ValueError('RGB-D timestamp mismatch')
            if not c['depth_aligned'] or any(m.header.frame_id != c['camera_frame'] for m in [color_msg, depth_msg, info]):
                raise ValueError('Need depth aligned in color optical frame')
            if info.distortion_model not in ['plumb_bob', 'rational_polynomial']:
                raise ValueError('Unsupported distortion model; rectify at camera publisher')
            if info.width != color_msg.width or info.height != color_msg.height:
                raise ValueError('Calibration resolution mismatch')
            if depth_msg.encoding not in ['16UC1', '32FC1']:
                raise ValueError('Depth must be 16UC1 mm or 32FC1 metres')
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
            if depth_msg.encoding == '16UC1':
                depth *= .001
            cube, marker = detect(color, depth, np.array(info.k).reshape(3, 3), np.array(info.d),
                                  c['marker_size_m'], c['marker_id'], c['cube_size_m'])
            if not c['handeye_validated']:
                raise ValueError('Detection succeeded; calibrated T_wrist_camera not yet validated')
            tf = self.buffer.lookup_transform(c['base_frame'], c['camera_frame'], Time.from_msg(color_msg.header.stamp))
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation
            base_camera = from_pose(pose)
            # Processing time is part of freshness, not just capture time.
            if (self.get_clock().now().nanoseconds * 1e-9 - min(stamps)) > c['max_frame_age_s']:
                raise ValueError('RGB-D processing exceeded freshness budget')
            msg = PoseArray()
            msg.header = color_msg.header
            msg.header.frame_id = c['base_frame']
            msg.poses = [to_pose(base_camera @ cube), to_pose(base_camera @ marker)]
            self.targets.publish(msg)
            status = {'ready': True, 'stamp_s': stamps[0], 'order': ['cube_center', 'table_marker']}
        except (ValueError, TransformException) as exc:
            status = {'ready': False, 'reason': str(exc)}
        out = String()
        out.data = json.dumps(status)
        self.status.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=default_config())
    args, ros_args = parser.parse_known_args()
    config = load_config(args.config)
    rclpy.init(args=ros_args)
    node = Perception(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
