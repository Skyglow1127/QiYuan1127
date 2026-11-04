#!/usr/bin/env python3
"""Measure ACT recorded-input HTTP calls through an existing SSH tunnel."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import io
import json
from pathlib import Path
import socket
import time

import numpy as np

IMAGE_KEYS = ('observation.images.global', 'observation.images.wrist')
INPUT_KEYS = ('observation.state', *IMAGE_KEYS)


def stats(values):
    values = np.asarray(values, dtype=np.float64)
    return {'count': len(values), 'median_seconds': float(np.median(values)),
            'min_seconds': float(np.min(values)), 'max_seconds': float(np.max(values)),
            'p95_seconds': float(np.percentile(values, 95))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18806)
    parser.add_argument('--mode', choices=('original_float_npz', 'fullres_uint8_npz'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--warmup', type=int, default=5)
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--source-fps', type=float, default=20)
    args = parser.parse_args()
    if args.warmup < 0 or args.samples < 1 or args.source_fps <= 0:
        parser.error('Nonnegative warmup, positive samples/FPS required.')
    read_start = time.perf_counter()
    input_bytes = args.input.read_bytes()
    with np.load(io.BytesIO(input_bytes), allow_pickle=False) as archive:
        original = {key: archive[key].copy() for key in INPUT_KEYS}
        episode, frame = int(archive['episode_index']), int(archive['frame_index'])
    input_read_decode = time.perf_counter() - read_start
    for key, value in original.items():
        expected = (3, 480, 640) if key in IMAGE_KEYS else (7,)
        if value.shape != expected or value.dtype != np.float32 or not np.isfinite(value).all():
            raise ValueError(f'Invalid original field {key}: {value.shape}/{value.dtype}')
        if key in IMAGE_KEYS and not (0 <= value.min() <= value.max() <= 1):
            raise ValueError(f'Image outside [0,1]: {key}')
    identity = {'input_mode': np.asarray(args.mode), 'source_episode': np.asarray(episode),
                'source_frame': np.asarray(frame),
                'source_relative_time_s': np.asarray(frame / args.source_fps),
                'source_timestamp_kind': np.asarray('recorded_frame_index_divided_by_fps')}

    def quantize(value):
        return np.rint(value * np.float32(255)).clip(0, 255).astype(np.uint8)

    conversion = {}
    if args.mode == 'fullres_uint8_npz':
        for key in IMAGE_KEYS:
            difference = np.abs(original[key] - quantize(original[key]).astype(np.float32) / np.float32(255))
            conversion[key] = {'shape': list(original[key].shape),
                               'max_abs_error_in_0_1': float(difference.max()),
                               'mean_abs_error_in_0_1': float(difference.mean()),
                               'geometry': 'unchanged; no resize, crop or JPEG',
                               'quantization': 'round(float32 * 255), uint8; server float32 / 255'}

    def encode():
        start = time.perf_counter()
        fields = dict(original)
        if args.mode == 'fullres_uint8_npz':
            for key in IMAGE_KEYS:
                fields[key] = quantize(original[key])
        transformed = time.perf_counter()
        fields.update(identity)
        stream = io.BytesIO()
        np.savez_compressed(stream, **fields)
        payload = stream.getvalue()
        encoded = time.perf_counter()
        return payload, {'quantize': transformed - start, 'npz_encode': encoded - transformed,
                         'input_encoding_total': encoded - start}

    connection = http.client.HTTPConnection(args.host, args.port, timeout=180)
    connect_start = time.perf_counter()
    connection.connect()
    connection.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    connect_seconds = time.perf_counter() - connect_start
    connection.request('GET', '/health')
    response = connection.getresponse()
    health = json.loads(response.read())
    if response.status != 200 or not health.get('ready'):
        raise RuntimeError(f'Server unhealthy: {health}')
    rows = []
    for index in range(1 + args.warmup + args.samples):
        phase = 'first' if index == 0 else ('warmup' if index <= args.warmup else 'measured')
        input_ready = time.perf_counter()
        payload, preparation = encode()
        post_start = time.perf_counter()
        connection.request('POST', '/infer', body=payload,
                           headers={'Content-Type': 'application/octet-stream',
                                    'Content-Length': str(len(payload))})
        written = time.perf_counter()
        response = connection.getresponse()
        headers_received = time.perf_counter()
        response_bytes = response.read()
        body_received = time.perf_counter()
        result = json.loads(response_bytes)
        if response.status != 200:
            raise RuntimeError(result)
        actions = np.asarray(result['actions'])
        if actions.shape != (100, 7) or not np.isfinite(actions).all():
            raise RuntimeError('Invalid returned actions.')
        if (result['source']['source_episode'] != episode or result['source']['source_frame'] != frame
                or result['input_mode'] != args.mode):
            raise RuntimeError('Returned request identity differs.')
        validated = time.perf_counter()
        seconds = {**preparation, 'request_write': written - post_start,
                   'wait_response_headers': headers_received - written,
                   'read_response_body': body_received - headers_received,
                   'post_roundtrip': body_received - post_start,
                   'json_decode_validate': validated - body_received,
                   'input_ready_to_validated_actions': validated - input_ready}
        rows.append({'index': index, 'phase': phase, 'request_bytes': len(payload),
                     'response_bytes': len(response_bytes), 'client_seconds': seconds,
                     'server_result': result})
        print(json.dumps({'index': index, 'phase': phase, 'client_seconds': seconds,
                          'server_seconds': result['server_seconds']}), flush=True)
    connection.close()
    measured = [row for row in rows if row['phase'] == 'measured']
    report = {'created_utc': datetime.now(timezone.utc).isoformat(), 'robot_io': False,
              'mode': args.mode, 'source_capture_wall_time': None,
              'input': str(args.input.resolve()), 'input_sha256': hashlib.sha256(input_bytes).hexdigest(),
              'source_episode': episode, 'source_frame': frame,
              'input_read_decode_seconds': input_read_decode, 'connect_seconds': connect_seconds,
              'image_conversion': conversion, 'server': health,
              'first_request': rows[0], 'warmup_count': args.warmup,
              'warm_summary': {
                  'client': {key: stats([row['client_seconds'][key] for row in measured])
                             for key in measured[0]['client_seconds']},
                  'server': {key: stats([row['server_result']['server_seconds'][key] for row in measured])
                             for key in measured[0]['server_result']['server_seconds']}},
              'timing_note': 'Input arrays ready in RAM -> actual per-request quantization/NPZ encoding '
                             '-> POST -> cloud decode, pre/model/post -> JSON return and finite/shape '
                             'validation. Disk read, live capture, feedback and actuation excluded. '
                             'Unknown recorded capture wallclock is not replaced by request time. '
                             'First request per client mode and global model-cold flag are distinct. '
                             'Keep-alive HTTP; TCP_NODELAY on both ends. No one-way network latency claim.',
              'results': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')


if __name__ == '__main__':
    main()
