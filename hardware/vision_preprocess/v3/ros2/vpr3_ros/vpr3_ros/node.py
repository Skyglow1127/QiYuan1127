"""Publish measured VPR3 RGB888 responses; backend and source never fall back."""
import argparse
import hashlib
import json
import time
import urllib.parse
import urllib.request
from array import array
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .metrics import distribution, observed_rate, server_stage_distributions


class LetterboxNode(Node):
    def __init__(self, args):
        super().__init__('vpr3_letterbox')
        self.args, self.last, self.records, self.errors = args, -1, [], []
        self.done = False
        self.source_kind = 'recorded_image' if args.source == 'recorded' else 'live_camera'
        with urllib.request.urlopen(args.url.rstrip('/')+'/status', timeout=5) as response:
            self.service_status = json.load(response)
        if (self.service_status.get('schema') != 'vpr3-service-v1'
                or self.service_status.get('source_kind') != self.source_kind
                or args.backend not in self.service_status.get('backends', [])):
            raise ValueError('VPR3 service/source/backend mismatch; fallback forbidden')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.image = self.create_publisher(Image, '/vpr3/image', qos)
        self.metadata = self.create_publisher(String, '/vpr3/geometry', qos)
        self.create_timer(1/args.rate, self.tick)

    def tick(self):
        if self.done:
            return
        if self.args.wait_subscriber and not self.records and (
                not self.image.get_subscription_count() or not self.metadata.get_subscription_count()):
            return
        try:
            begin, stamp = time.perf_counter_ns(), time.time_ns()
            query = urllib.parse.urlencode(dict(role=self.args.role, backend=self.args.backend,
                width=self.args.width, height=self.args.height, after=self.last, rounding='floor'))
            with urllib.request.urlopen(self.args.url.rstrip('/')+'/process?'+query, timeout=5) as response:
                if response.status == 204:
                    if self.args.source == 'recorded':
                        raise ValueError('Recorded service unexpectedly returned no frame')
                    return
                raw = response.read()
                meta = json.loads(response.headers['X-VPR-Metadata'])
            received = time.perf_counter_ns()
            g = meta['geometry']
            if (meta.get('schema') != 'vpr3-image-v1' or meta['backend'] != self.args.backend
                    or meta['source_kind'] != self.source_kind or meta['role'] != self.args.role):
                raise ValueError('Response identity does not match requested VPR3 source/backend')
            if (g['width'] != self.args.width or g['height'] != self.args.height
                    or g['pad'] != 0 or g['rounding'] != 'floor'
                    or meta['encoding'] != 'rgb8' or meta['network_payload_format'] != 'rgb8'
                    or len(raw) != g['width']*g['height']*3
                    or meta['network_payload_bytes'] != len(raw)):
                raise ValueError('RGB888 or Pi05 geometry contract mismatch')
            if hashlib.sha256(raw).hexdigest() != meta['output_sha256']:
                raise ValueError('HTTP payload does not match service hash')
            if self.args.source == 'live' and meta['source_frame_index'] <= self.last:
                raise ValueError('Live frame index did not advance')
            validated = time.perf_counter_ns()
            msg = Image()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.header.frame_id = ('recorded_' if self.args.source == 'recorded' else 'kr260_')+self.args.role+'_color_optical_frame'
            msg.height, msg.width, msg.encoding, msg.is_bigendian = g['height'], g['width'], 'rgb8', 0
            msg.step, msg.data = msg.width*3, array('B', raw)
            prepared = time.perf_counter_ns()
            self.image.publish(msg)
            published = time.perf_counter_ns()
            meta.update(host_request_start_monotonic_ns=begin, ros_stamp_ns=stamp,
                ros_stamp_semantics='host request start, not exposure timestamp',
                host_timing_ns={'request_response': received-begin,
                    'response_validation': validated-received, 'message_copy': prepared-validated,
                    'image_publish': published-prepared, 'request_to_publish': published-begin,
                    'transport_and_http_overhead': received-begin-meta['timing_ns']['server_total']},
                transport_timing_semantics='request_response includes server; overhead subtracts server duration and also includes HTTP scheduling, not isolated network latency',
                published_monotonic_ns=published)
            message = String()
            message.data = json.dumps(meta, separators=(',', ':'))
            self.metadata.publish(message)
            self.last = meta['source_frame_index']
            self.records.append(meta)
            self.done = len(self.records) >= self.args.frames
        except Exception as error:
            self.errors.append(str(error))
            self.get_logger().error(str(error))
            self.done = True

    def save(self):
        host_stages = sorted({key for record in self.records for key in record['host_timing_ns']})
        result = {'status': 'PUBLISHED' if len(self.records) == self.args.frames and not self.errors else 'FAILED',
            'backend': self.args.backend, 'role': self.args.role, 'source_kind': self.source_kind,
            'published_messages': len(self.records), 'service_status': self.service_status,
            'unique_source_frames': len({r['source_frame_index'] for r in self.records}),
            'observed_publish_fps': observed_rate([r['published_monotonic_ns'] for r in self.records]),
            'requested_publish_rate_hz': self.args.rate, 'errors': self.errors,
            'camera_exposure_included': False, 'model_action_regression': 'NOT_RUN',
            'server_stage_ms': server_stage_distributions(self.records),
            'host_stage_ms': {key: distribution([r['host_timing_ns'][key] for r in self.records]) for key in host_stages},
            'frames': self.records}
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.args.output.write_text(json.dumps(result, indent=2)+'\n')
        return result['status'] == 'PUBLISHED'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18768')
    parser.add_argument('--role', choices=['wrist', 'global'], default='wrist')
    parser.add_argument('--backend', choices=['software', 'pl'], default='software')
    parser.add_argument('--source', choices=['live', 'recorded'], required=True)
    parser.add_argument('--width', type=int, default=224)
    parser.add_argument('--height', type=int, default=224)
    parser.add_argument('--frames', type=int, default=60)
    parser.add_argument('--rate', type=float, default=60)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--wait-subscriber', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args, ros = parser.parse_known_args()
    if args.frames < 1 or args.rate <= 0 or args.timeout <= 0 or not (16 <= args.width <= 640 and 16 <= args.height <= 640):
        parser.error('frames/rate/timeout positive; canvas axes 16..640')
    rclpy.init(args=ros)
    node = None
    ok = False
    try:
        node = LetterboxNode(args)
        deadline = time.monotonic()+args.timeout
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not node.done:
            node.errors.append('Publisher deadline exceeded')
        node.image.wait_for_all_acked(Duration(seconds=2))
        node.metadata.wait_for_all_acked(Duration(seconds=2))
        ok = node.save()
    except Exception as error:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({'status': 'FAILED', 'source': args.source,
            'backend': args.backend, 'error': str(error)}, indent=2)+'\n')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    if not ok:
        raise SystemExit(1)
