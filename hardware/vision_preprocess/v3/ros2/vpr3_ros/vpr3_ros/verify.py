"""Verify actual ROS Images and metadata, including Pi05 borders and identity."""
import argparse
import hashlib
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .metrics import distribution, observed_rate


def require(condition, message):
    if not condition:
        raise ValueError(message)


class Verify(Node):
    def __init__(self, args):
        super().__init__('vpr3_verify')
        self.args, self.images, self.metadata, self.records = args, {}, {}, []
        self.seen, self.error = set(), None
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, '/vpr3/image', self.image, qos)
        self.create_subscription(String, '/vpr3/geometry', self.meta, qos)

    def image(self, message):
        try:
            stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            require(stamp not in self.images and stamp not in self.seen, 'Duplicate image stamp')
            self.images[stamp] = (message, time.perf_counter_ns())
            self.match(stamp)
        except Exception as error:
            self.error = str(error)

    def meta(self, message):
        try:
            received = time.perf_counter_ns()
            data = json.loads(message.data)
            stamp = data['ros_stamp_ns']
            require(stamp not in self.metadata and stamp not in self.seen, 'Duplicate metadata stamp')
            self.metadata[stamp] = (data, received)
            self.match(stamp)
        except Exception as error:
            self.error = str(error)

    def match(self, stamp):
        if stamp not in self.images or stamp not in self.metadata:
            return
        message, received = self.images.pop(stamp)
        meta, meta_received = self.metadata.pop(stamp)
        copy_begin = time.perf_counter_ns()
        raw = bytes(message.data)
        copied = time.perf_counter_ns()
        g = meta['geometry']
        source = 'recorded_image' if self.args.source == 'recorded' else 'live_camera'
        require(meta['schema'] == 'vpr3-image-v1' and meta['backend'] == self.args.backend
                and meta['source_kind'] == source and meta['role'] == self.args.role, 'Wrong source/backend/schema/role')
        require(message.encoding == 'rgb8' and not message.is_bigendian
                and message.width == g['width'] == self.args.width
                and message.height == g['height'] == self.args.height
                and message.step == message.width*3 and len(raw) == message.step*message.height,
                'Image encoding, dimensions or byte count mismatch')
        require(hashlib.sha256(raw).hexdigest() == meta['output_sha256'], 'ROS image and metadata hash differ')
        ratio = max(g['iw']/g['width'], g['ih']/g['height'])
        rw, rh = int(g['iw']/ratio), int(g['ih']/ratio)
        require(g['pad'] == 0 and g['rounding'] == 'floor' and g['rw'] == rw and g['rh'] == rh,
                'Pi05 floor/black geometry mismatch')
        require(g['left'] == (g['width']-rw)//2 and g['top'] == (g['height']-rh)//2
                and g['right'] == g['width']-rw-g['left'] and g['bottom'] == g['height']-rh-g['top'],
                'Pi05 centered padding mismatch')
        for y in range(message.height):
            row = raw[y*message.step:(y+1)*message.step]
            if y < g['top'] or y >= g['top']+rh:
                require(row == bytes(message.step), 'Nonzero border row')
            else:
                require(row[:g['left']*3] == bytes(g['left']*3), 'Nonzero left border')
                require(row[(g['left']+rw)*3:] == bytes(g['right']*3), 'Nonzero right border')
        prefix = 'recorded_' if self.args.source == 'recorded' else 'kr260_'
        require(message.header.frame_id == prefix+self.args.role+'_color_optical_frame', 'Frame-id source label mismatch')
        require(isinstance(meta['request_index'], int) and meta['request_index'] >= 0, 'Invalid request identity')
        duration = received-meta['host_request_start_monotonic_ns']
        require(duration >= 0, 'Host monotonic timing mismatch')
        self.seen.add(stamp)
        self.records.append({'source_frame_index': meta['source_frame_index'], 'request_index': meta['request_index'],
            'source_kind': source, 'source_jpeg_sha256': meta['source_jpeg_sha256'],
            'header_frame_id': message.header.frame_id, 'geometry': g, 'received_monotonic_ns': received,
            'request_to_subscriber_ns': duration,
            'request_to_paired_delivery_ns': max(received, meta_received)-meta['host_request_start_monotonic_ns'],
            'subscriber_copy_ns': copied-copy_begin, 'subscriber_validation_ns': time.perf_counter_ns()-copied,
            'output_sha256': meta['output_sha256']})
        if len(self.records) == 1:
            self.args.output.with_suffix('.rgb').write_bytes(raw)

    def save(self):
        source_unique = len({frame['source_frame_index'] for frame in self.records})
        request_unique = len({frame['request_index'] for frame in self.records})
        identity_ok = request_unique == self.args.frames
        if self.args.source == 'live':
            identity_ok = identity_ok and source_unique == self.args.frames
        ok = len(self.records) == self.args.frames and identity_ok and self.error is None
        report = {'status': 'ROS_IMAGE_PASS' if ok else 'FAIL', 'backend': self.args.backend,
            'role': self.args.role, 'source_kind': 'recorded_image' if self.args.source == 'recorded' else 'live_camera',
            'matched_messages': len(self.records), 'unique_camera_frames': source_unique if self.args.source == 'live' else 0,
            'unique_source_frames': source_unique, 'unique_request_indices': request_unique,
            'unique_source_jpegs': len({frame['source_jpeg_sha256'] for frame in self.records}),
            'error': self.error, 'unmatched_images': len(self.images), 'unmatched_metadata': len(self.metadata),
            'observed_subscriber_fps': observed_rate([frame['received_monotonic_ns'] for frame in self.records]),
            'latency': {key: distribution([frame[key] for frame in self.records]) for key in (
                'request_to_subscriber_ns', 'request_to_paired_delivery_ns', 'subscriber_copy_ns', 'subscriber_validation_ns')},
            'model_action_regression': 'NOT_RUN', 'frames': self.records}
        self.args.output.write_text(json.dumps(report, indent=2)+'\n')
        return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=int, default=60)
    parser.add_argument('--backend', choices=['software', 'pl'], default='software')
    parser.add_argument('--source', choices=['live', 'recorded'], required=True)
    parser.add_argument('--role', choices=['wrist', 'global'], default='wrist')
    parser.add_argument('--width', type=int, default=224)
    parser.add_argument('--height', type=int, default=224)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--output', type=Path, required=True)
    args, ros = parser.parse_known_args()
    if args.frames < 1 or args.timeout <= 0:
        parser.error('frames and timeout must be positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros)
    node = Verify(args)
    deadline = time.monotonic()+args.timeout
    try:
        while len(node.records) < args.frames and node.error is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(node.records) < args.frames and node.error is None:
            node.error = 'Verifier deadline exceeded'
    finally:
        ok = node.save()
        node.destroy_node()
        rclpy.shutdown()
    if not ok:
        raise SystemExit(1)
