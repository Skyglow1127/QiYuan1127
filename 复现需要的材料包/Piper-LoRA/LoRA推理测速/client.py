#!/usr/bin/env python3
"""Benchmark a full recorded dual-camera PI05 request through the local inference service.

Read disk once, then actually encode the observation on every measured request.
No camera, robot, CAN, GUI or actuator operations are present.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import http.client
import io
import json
from pathlib import Path
import statistics
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=18805)
    parser.add_argument('--repeats', type=int, default=6)
    parser.add_argument('--mode', choices=('original_float_npz', 'resized_uint8_npz'),
                        default='original_float_npz')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--source-fps', type=float, default=20)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.host != "127.0.0.1":
        parser.error("The benchmark connects only to 127.0.0.1")
    if args.repeats < 4 or args.source_fps <= 0:
        parser.error('Use at least 4 repeats (first request + 3 warm), with positive FPS.')
    import numpy as np
    disk_start = time.perf_counter()
    input_bytes = args.input.read_bytes()
    with np.load(io.BytesIO(input_bytes), allow_pickle=False) as archive:
        fields = ('observation.state', 'observation.images.global', 'observation.images.wrist', 'task')
        original_fields = {key: archive[key].copy() for key in fields}
        episode, frame = int(archive['episode_index']), int(archive['frame_index'])
    disk_read_decode_seconds = time.perf_counter() - disk_start
    import_start = time.perf_counter()
    if args.mode == 'resized_uint8_npz':
        import torch
        from lerobot.policies.common.vla_utils import resize_with_pad_torch
        torch.set_num_threads(8)
    runtime_import_seconds = time.perf_counter() - import_start
    source_fields = {
        'source_episode': np.asarray(episode), 'source_frame': np.asarray(frame),
        'source_relative_time_s': np.asarray(frame / args.source_fps),
        'source_timestamp_kind': np.asarray('recorded_frame_index_divided_by_fps'),
        'input_mode': np.asarray(args.mode)}

    def resized_image(array):
        image = torch.from_numpy(array).to(torch.float32)
        resized = resize_with_pad_torch(
            image.unsqueeze(0).permute(0, 2, 3, 1), 224, 224).permute(0, 3, 1, 2)[0]
        return resized, torch.round(resized * 255.0).clamp(0, 255).to(torch.uint8)

    def prepare_payload():
        began = time.perf_counter()
        payload_fields = dict(original_fields)
        if args.mode == 'resized_uint8_npz':
            for key in ('observation.images.global', 'observation.images.wrist'):
                _, quantized = resized_image(original_fields[key])
                payload_fields[key] = quantized.numpy()
        transform_end = time.perf_counter()
        payload_fields.update(source_fields)
        stream = io.BytesIO()
        encode_start = time.perf_counter()
        np.savez_compressed(stream, **payload_fields)
        payload = stream.getvalue()
        end = time.perf_counter()
        return payload, {
            'resize_quantize': transform_end - began,
            'npz_encode': end - encode_start,
            'input_encoding_total': end - began}

    # Quantization audit runs outside all benchmark intervals.
    image_conversion = {}
    if args.mode == 'resized_uint8_npz':
        for key in ('observation.images.global', 'observation.images.wrist'):
            resized, quantized = resized_image(original_fields[key])
            difference = (resized - quantized.to(torch.float32) / 255.0).abs()
            image_conversion[key] = {
                'original_shape': list(original_fields[key].shape),
                'wire_shape': list(quantized.shape), 'wire_dtype': 'uint8',
                'max_abs_error_in_0_1': float(difference.max()),
                'mean_abs_error_in_0_1': float(difference.mean()),
                'max_abs_error_after_model_minus1_1_mapping': float(difference.max()) * 2,
                'resize': 'canonical lerobot resize_with_pad_torch; '
                          'bilinear align_corners=False, centered black padding',
                'quantization': 'round(resized_float32 * 255), uint8; server float32 / 255',
                'comparison_note': 'Compared to canonical CPU-resized float image. '
                                   'Not identical to original full-resolution float input; '
                                   'CPU/GPU resize floating-point differences are not measured.'}
    if args.prepare_only:
        payload, preparation = prepare_payload()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.with_suffix('.request.npz').write_bytes(payload)
        report = {'mode': args.mode, 'request_bytes': len(payload), 'robot_io': False,
                  'disk_read_decode_seconds': disk_read_decode_seconds,
                  'runtime_import_seconds': runtime_import_seconds,
                  'preparation_seconds': preparation, 'image_conversion': image_conversion,
                  'source_episode': episode, 'source_frame': frame,
                  'source_capture_wall_time': None}
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(report, ensure_ascii=False), flush=True)
        return

    connection = http.client.HTTPConnection(args.host, args.port, timeout=180)
    connect_start = time.perf_counter()
    connection.connect()
    connect_seconds = time.perf_counter() - connect_start
    connection.request('GET', '/health')
    response = connection.getresponse()
    health = json.loads(response.read())
    if response.status != 200 or not health.get('ready'):
        raise RuntimeError(f'Server is not ready: {health}')
    results = []
    for repeat in range(args.repeats):
        # Input arrays are ready in memory at this boundary. This is NOT a
        # camera-capture timestamp. Resize/quantize/encode really run each time.
        input_ready = time.perf_counter()
        payload, preparation = prepare_payload()
        sent_unix_ns = time.time_ns()
        post_start = time.perf_counter()
        connection.request('POST', '/infer', body=payload,
                           headers={'Content-Type': 'application/octet-stream',
                                    'Content-Length': str(len(payload)),
                                    'X-Client-Sent-Unix-Ns': str(sent_unix_ns)})
        written = time.perf_counter()
        response = connection.getresponse()
        headers_received = time.perf_counter()
        raw_response = response.read()
        body_received = time.perf_counter()
        result = json.loads(raw_response)
        if response.status != 200:
            raise RuntimeError(result)
        actions = np.asarray(result['actions'])
        if actions.shape != (50, 7) or not np.isfinite(actions).all():
            raise RuntimeError('Invalid returned action array.')
        if result['source']['source_episode'] != episode or result['source']['source_frame'] != frame:
            raise RuntimeError('Returned source identity differs.')
        if result['input_mode'] != args.mode:
            raise RuntimeError('Server input mode differs.')
        validated = time.perf_counter()
        roundtrip = body_received - post_start
        processing = sum(result['server_seconds'][key] for key in
                         ('unpack_validate', 'preprocess', 'model', 'postprocess', 'actions_to_list'))
        measured = {
            'repeat': repeat, 'first_request_in_this_client_run': repeat == 0,
            'request_bytes': len(payload), 'response_bytes': len(raw_response),
            'client_sent_unix_ns': sent_unix_ns,
            'client_seconds': {
                **preparation,
                'request_write': written - post_start,
                'wait_for_response_headers': headers_received - written,
                'read_response_body': body_received - headers_received,
                'roundtrip': roundtrip,
                'json_decode_and_validate': validated - body_received,
                'input_ready_to_validated_actions': validated - input_ready,
                'network_and_other_overhead_residual': roundtrip - processing,
                'residual_excluding_server_body_read_and_processing':
                    roundtrip - result['server_seconds']['before_response_serialization']},
            'server_result': result}
        results.append(measured)
        print(json.dumps({'repeat': repeat,
                          'global_model_cold': result['cold_prediction'],
                          'first_request_for_mode': result['first_request_for_mode'],
                          'roundtrip_seconds': roundtrip,
                          'input_ready_to_validated_actions_seconds': validated - input_ready,
                          'server_model_seconds': result['server_seconds']['model']}), flush=True)
    connection.close()
    # Exclude the mode's first request and this client's first request. Only the
    # server's first-ever prediction is model-cold; another encoding is not cold.
    warm = [item for item in results if item['repeat'] > 0
            and not item['server_result']['first_request_for_mode']
            and not item['server_result']['cold_prediction']]
    report = {
        'created_utc': datetime.now(timezone.utc).isoformat(), 'robot_io': False,
        'mode': args.mode, 'image_conversion': image_conversion,
        'input': str(args.input.resolve()),
        'input_file_sha256': hashlib.sha256(input_bytes).hexdigest(),
        'disk_read_decode_seconds': disk_read_decode_seconds,
        'runtime_import_seconds': runtime_import_seconds,
        'transport': 'HTTP on 127.0.0.1',
        'request_bytes': len(payload),
        'local_connection_seconds': connect_seconds, 'server': health,
        'source_capture_wall_time': None,
        'source_note': 'Recorded source capture wallclock unknown. Request send timestamp '
                       'and input-ready timing boundary are not camera capture timestamps.',
        'timing_note': 'Disk read once outside loop. Each input-ready-to-action interval '
                       'includes actual resize/quantize/NPZ encode, POST, JSON decode and validation. '
                       'Global model cold and first request for encoding mode are distinguished. '
                       'Network residue retains upload/body-read time but also includes local HTTP, '
                       'scheduling, response serialization and logging. No one-way claim. '
                       'Live capture, GUI, feedback, control and actuators are excluded.',
        'warm_summary': {
            'count': len(warm),
            'roundtrip_median_seconds': statistics.median(x['client_seconds']['roundtrip'] for x in warm),
            'roundtrip_min_seconds': min(x['client_seconds']['roundtrip'] for x in warm),
            'roundtrip_max_seconds': max(x['client_seconds']['roundtrip'] for x in warm),
            'input_ready_to_validated_actions_median_seconds': statistics.median(
                x['client_seconds']['input_ready_to_validated_actions'] for x in warm),
            'input_ready_to_validated_actions_min_seconds': min(
                x['client_seconds']['input_ready_to_validated_actions'] for x in warm),
            'input_ready_to_validated_actions_max_seconds': max(
                x['client_seconds']['input_ready_to_validated_actions'] for x in warm),
            'model_median_seconds': statistics.median(x['server_result']['server_seconds']['model'] for x in warm)},
        'results': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print('Saved:', args.output)


if __name__ == '__main__':
    main()
