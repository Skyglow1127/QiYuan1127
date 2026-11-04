#!/usr/bin/env python3
"""Loopback-only VPR3 service for existing camera URLs or explicit recordings."""
import argparse
import hashlib
import itertools
import json
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from time import perf_counter_ns, process_time_ns
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
from geometry import geometry, cpu


def main(args):
    cv2.setNumThreads(1)
    sequence = itertools.count()
    if args.samples:
        for role in ('wrist', 'global'):
            if not (args.samples / (role + '.jpg')).is_file():
                raise ValueError('Recorded sample missing: ' + role)
    cameras = {'wrist': args.wrist_url, 'global': args.global_url}
    transport = None
    if args.enable_pl:
        from transport import PL
        transport = PL()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            try:
                parsed = urlsplit(self.path)
                query = parse_qs(parsed.query)
                if parsed.path == '/status':
                    raw = json.dumps({'schema': 'vpr3-service-v1',
                        'roles': list(cameras), 'backends': ['software'] + (['pl'] if transport else []),
                        'source_kind': 'recorded_image' if args.samples else 'live_camera',
                        'input': 'recorded JPEG fixtures' if args.samples else 'explicit existing camera URLs',
                        'opencv': cv2.__version__, 'default_canvas': [224, 224],
                        'ps_baseline': 'OpenCV uint8, not official torch float',
                        'model_action_regression': 'NOT_RUN'}).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(raw)))
                    self.end_headers()
                    self.wfile.write(raw)
                    return
                if parsed.path != '/process':
                    self.send_error(404)
                    return
                role = query.get('role', ['wrist'])[0]
                if role not in cameras:
                    raise ValueError('Unknown camera role')
                backend = query.get('backend', ['software'])[0]
                width = int(query.get('width', ['224'])[0])
                height = int(query.get('height', ['224'])[0])
                if not 16 <= width <= 640 or not 16 <= height <= 640:
                    raise ValueError('Canvas axes must be 16..640')
                cpu_begin = process_time_ns()
                begin = perf_counter_ns()
                if args.samples:
                    jpeg = (args.samples / (role + '.jpg')).read_bytes()
                    index, source_age = 0, None
                else:
                    with urllib.request.urlopen(cameras[role], timeout=4) as response:
                        index = int(response.headers['X-Frame-Index'])
                        source_age = float(response.headers['X-Board-Age-Ms'])
                        if index <= int(query.get('after', ['-1'])[0]):
                            self.send_response(204)
                            self.send_header('Content-Length', '0')
                            self.end_headers()
                            return
                        jpeg = response.read()
                fetched = perf_counter_ns()
                bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    raise ValueError('Camera JPEG decode failed')
                image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                decoded = perf_counter_ns()
                g = geometry(image.shape[1], image.shape[0], width, height,
                             rounding=query.get('rounding', ['floor'])[0], pad=0)
                if backend == 'software':
                    out, stages = cpu(image, g)
                elif backend == 'pl' and transport is not None:
                    out, stages = transport.run(image, g)
                else:
                    raise ValueError('Requested backend unavailable; no implicit fallback')
                processed = perf_counter_ns()
                raw = out.tobytes()
                serialized = perf_counter_ns()
                output_hash = hashlib.sha256(raw).hexdigest()
                input_hash = hashlib.sha256(jpeg).hexdigest()
                hashed = perf_counter_ns()
                metadata = {'schema': 'vpr3-image-v1', 'role': role,
                    'source_frame_index': index, 'backend': backend, 'encoding': 'rgb8',
                    'source_kind': 'recorded_image' if args.samples else 'live_camera',
                    'request_index': next(sequence), 'source_jpeg_sha256': input_hash,
                    'geometry': g.metadata(), 'source_age_at_camera_fetch_ms': source_age,
                    'output_sha256': output_hash, 'network_payload_format': 'rgb8',
                    'network_payload_bytes': len(raw), 'board_camera_jpeg_bytes': len(jpeg),
                    'ps_baseline': 'OpenCV uint8, not official torch float',
                    'model_action_regression': 'NOT_RUN',
                    'timing_ns': {'camera_fetch': fetched-begin,
                                  'jpeg_decode_and_rgb': decoded-fetched,
                                  'preprocess_wall': processed-decoded,
                                  'output_serialize': serialized-processed,
                                  'integrity_hash': hashed-serialized,
                                  'server_total': hashed-begin,
                                  'server_cpu_time': process_time_ns()-cpu_begin, **stages}}
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(len(raw)))
                self.send_header('X-VPR-Metadata', json.dumps(metadata, separators=(',', ':')))
                self.end_headers()
                self.wfile.write(raw)
            except (ValueError, KeyError) as error:
                self.send_error(400, str(error))
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as error:
                self.send_error(503, str(error))

        def log_message(self, *unused):
            pass

    try:
        with HTTPServer(('127.0.0.1', args.port), Handler) as server:
            print('VPR3_SERVICE_READY', args.port, flush=True)
            server.serve_forever()
    finally:
        if transport:
            transport.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18768)
    parser.add_argument('--enable-pl', action='store_true')
    parser.add_argument('--samples', type=Path)
    parser.add_argument('--wrist-url', default='http://127.0.0.1:18760/color.jpg')
    parser.add_argument('--global-url', default='http://127.0.0.1:18761/color.jpg')
    main(parser.parse_args())
