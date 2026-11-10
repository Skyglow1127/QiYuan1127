#!/usr/bin/env python3
"""Persistent, hardware-attested red foreground backend for the grasp workflow.

The output is a 2D foreground observation, not a pose or motion permission.
Run hardware commands through /home/aup/NPU/ryzenai.sh to isolate ROS libraries.
The HTTP client imports only the standard library and can run in the ROS process.
"""
import argparse
import base64
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import importlib.util
import json
from pathlib import Path
import threading
import time
import urllib.request

SOURCE = Path('/home/aup/NPU/integration/20261023/red_cube_npu')
MODEL = SOURCE / 'model-int8.onnx'
MAX_IMAGE_BYTES = 4 * 1024 * 1024


def sha256_bytes(raw):
    return hashlib.sha256(raw).hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def _load_helpers():
    spec = importlib.util.spec_from_file_location('grasp_red_foreground_helpers', SOURCE / 'benchmark_segmentation.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_metadata(metadata, raw):
    """Only validate what the producer can attest; replay never claims freshness."""
    if metadata.get('image_sha256') != sha256_bytes(raw):
        raise ValueError('image_sha256 missing or does not bind the submitted bytes')
    kind = metadata.get('source_kind')
    if kind not in ('saved_image', 'live_camera'):
        raise ValueError('source_kind must be saved_image or live_camera')
    if kind == 'live_camera':
        if metadata.get('camera_serial') not in ('CC1365301EY', 'CC1365302K0'):
            raise ValueError('unexpected camera identity')
        if not isinstance(metadata.get('frame_index'), int) or metadata['frame_index'] < 0:
            raise ValueError('missing live frame index')
        age = metadata.get('board_age_at_fetch_ms')
        if not isinstance(age, (int, float)) or not 0 <= age <= 250:
            raise ValueError('invalid or stale input frame')
        received = metadata.get('received_host_monotonic')
        elapsed = time.monotonic() - received if isinstance(received, (int, float)) else float('inf')
        if not 0 <= elapsed <= .8:
            raise ValueError('missing/expired host-monotonic timestamp; local-host producer required')


class RedForegroundBackend:
    """One initialized ORT session. Calls are serialized by the HTTP server."""
    def __init__(self, backend, evidence_dir, model=MODEL):
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.helpers = _load_helpers()
        self.backend, self.model = backend, Path(model).resolve()
        self.evidence_dir = Path(evidence_dir).resolve()
        self.evidence_dir.mkdir(parents=True, exist_ok=False)
        (self.evidence_dir / 'implementation_snapshot.py').write_bytes(Path(__file__).read_bytes())
        (self.evidence_dir / 'helper_snapshot.py').write_bytes((SOURCE / 'benchmark_segmentation.py').read_bytes())
        self.model_hash = sha256_bytes(self.model.read_bytes())
        cv2.setNumThreads(1)
        self.session = None
        self.sequence = 0
        started = time.perf_counter()
        self.hardware = {'confirmed': False, 'reason': 'CPU backend explicitly selected'}
        if backend in ('cpu', 'npu'):
            import os
            os.environ['XLNX_ONNX_EP_REPORT_FILE'] = 'vitisai_ep_report.json'
            self.session, seconds = self.helpers.session_for(
                self.model, backend, self.evidence_dir, SOURCE / 'bf16_config.json', precision='int8')
            self.input_name = self.session.get_inputs()[0].name
            # Warmup produces actual device execution evidence before readiness.
            prepared = self.helpers.preprocess(np.zeros((480, 640, 3), np.uint8))
            for _ in range(5):
                self.session.run(None, {self.input_name: prepared})
            self.profile = self.session.end_profiling()
            if backend == 'npu':
                self.hardware = self.helpers.npu_evidence(self.evidence_dir, self.profile)
                if not self.hardware['confirmed']:
                    raise RuntimeError('NPU allocation AND executed fused event not confirmed')
        elif backend != 'hsv':
            raise ValueError('backend must be npu, cpu or hsv')
        self.startup_ms = (time.perf_counter() - started) * 1000
        self.evidence = dict(schema='grasp-foreground-session-v1', backend=backend,
            created_at=datetime.now(timezone.utc).isoformat(), model=str(self.model), model_sha256=self.model_hash,
            implementation_sha256=sha256_bytes(Path(__file__).read_bytes()),
            implementation_snapshot=str(self.evidence_dir / 'implementation_snapshot.py'),
            helper_sha256=sha256_bytes((SOURCE / 'benchmark_segmentation.py').read_bytes()),
            providers=self.session.get_providers() if self.session else [], hardware_execution=self.hardware,
            automatic_session_fallback=False, cpu_partition_ops_allowed=True,
            startup_ms=self.startup_ms, profile_scope='five startup inferences; reused session keeps profiling off during measured requests',
            output_contract='red foreground mask/ROI; no ordered corners or 3D pose', motion_commands=0)
        self.evidence_path = self.evidence_dir / 'session_evidence.json'
        save_json(self.evidence_path, self.evidence)
        self.evidence_hash = sha256_bytes(self.evidence_path.read_bytes())

    def infer_bytes(self, raw, metadata, queue_ms=0.0, preprocessed=False):
        if not raw or len(raw) > MAX_IMAGE_BYTES:
            raise ValueError('empty or oversized image')
        validate_metadata(metadata, raw)
        wall_start, cpu_start = time.perf_counter(), time.process_time()
        ticks = [wall_start]
        image_shape = (480, 640, 3)
        if preprocessed:
            if self.backend == 'hsv':
                raise ValueError('preprocessed input is defined only for the model backends')
            if len(raw) != 240*320*3 or metadata.get('input_format') != 'rgb_u8_320x240':
                raise ValueError('expected exactly 230400 bytes, RGB uint8 HWC 320x240')
            if metadata.get('resize_transform') != {'source_width': 640, 'source_height': 480, 'target_width': 320, 'target_height': 240, 'algorithm': 'area_2x2'}:
                raise ValueError('missing/unsupported resize transform')
            original_hash = metadata.get('original_image_sha256', '')
            if len(original_hash) != 64 or any(c not in '0123456789abcdef' for c in original_hash):
                raise ValueError('missing original image identity')
            image = self.np.frombuffer(raw, self.np.uint8).reshape(240, 320, 3)
        else:
            image = self.cv2.imdecode(self.np.frombuffer(raw, self.np.uint8), self.cv2.IMREAD_COLOR)
            if image is None or image.shape != image_shape:
                raise ValueError('expected 640x480 BGR camera image')
        ticks.append(time.perf_counter())
        if self.session:
            tensor = (self.np.ascontiguousarray(image.transpose(2, 0, 1)[None], dtype=self.np.float32)
                      / self.np.float32(255.0)) if preprocessed else self.helpers.preprocess(image)
            ticks.append(time.perf_counter())
            logits = self.session.run(None, {self.input_name: tensor})[0]
            ticks.append(time.perf_counter())
            # Count all foreground components before the historical helper keeps
            # only the largest one. Otherwise a second cube disappears silently.
            raw_binary = self.np.argmax(logits[0], axis=0).astype(self.np.uint8)
            component_total, _, component_stats, _ = self.cv2.connectedComponentsWithStats(raw_binary, connectivity=8)
            component_areas = [int(v) for v in component_stats[1:, self.cv2.CC_STAT_AREA]]
            largest_area = max(component_areas, default=0)
            significant_count = sum(area > .2*largest_area for area in component_areas)
            mask, geometry = self.helpers.postprocess(logits, image_shape)
        else:
            ticks.extend([time.perf_counter(), time.perf_counter()])
            mask, geometry = self.helpers.hsv_pipeline(image)
            # HSV is a diagnostic baseline; its existing helper also selects one
            # component, so compute the same multi-component evidence separately.
            hsv = self.cv2.cvtColor(image, self.cv2.COLOR_BGR2HSV)
            raw_binary = self.np.zeros(image.shape[:2], self.np.uint8)
            for low, high in self.helpers.HSV_RANGES:
                raw_binary |= self.cv2.inRange(hsv, self.np.array(low,self.np.uint8), self.np.array(high,self.np.uint8))
            component_total, _, component_stats, _ = self.cv2.connectedComponentsWithStats(raw_binary, connectivity=8)
            component_areas = [int(v) for v in component_stats[1:, self.cv2.CC_STAT_AREA]]
            largest_area = max(component_areas, default=0)
            significant_count = sum(area > .2*largest_area for area in component_areas)
        ticks.append(time.perf_counter())
        ok, png = self.cv2.imencode('.png', mask * 255)
        if not ok:
            raise RuntimeError('mask encode failed')
        mask_raw = png.tobytes()
        mask_b64 = base64.b64encode(mask_raw).decode('ascii')
        ticks.append(time.perf_counter())
        self.sequence += 1
        timing = dict(zip(('decode_ms', 'preprocess_ms', 'inference_ms', 'postprocess_ms', 'mask_encode_ms'),
                         [(b-a)*1000 for a,b in zip(ticks, ticks[1:])]))
        timing.update(queue_ms=queue_ms, service_compute_ms=(ticks[-1]-ticks[0])*1000,
                      process_cpu_ms=(time.process_time()-cpu_start)*1000)
        age = None
        if metadata['source_kind'] == 'live_camera':
            age = metadata['board_age_at_fetch_ms'] + (time.monotonic()-metadata['received_host_monotonic'])*1000
        rejection = geometry['observation_rejection_reason']
        if significant_count > 1:
            rejection = 'multiple_comparable_foreground_components'
        if age is not None and age > 250:
            rejection = 'result_frame_age_exceeds_250ms'
        # Geometric consistency, identity and motion checks belong to the consumer.
        return dict(schema='grasp-foreground-v1', sequence=self.sequence, backend=self.backend,
            status='REJECTED' if rejection else 'FOREGROUND_CANDIDATE', rejection_reason=rejection,
            image_sha256=metadata.get('original_image_sha256', metadata['image_sha256']) if preprocessed else metadata['image_sha256'],
            submitted_payload_sha256=metadata['image_sha256'], frame=metadata,
            preprocessed_input=preprocessed,
            preprocessor_evidence=metadata.get('preprocessor_evidence'),
            preprocessor_evidence_verification='not verified by model service; consumer must validate producer evidence' if preprocessed else None,
            result_frame_age_upper_estimate_ms=age,
            frame_age_scope='board header age plus elapsed on this host; producer must account for camera transport',
            image_width=640, image_height=480, mask_png_base64=mask_b64,
            significant_component_count=significant_count,
            foreground_component_count=int(component_total-1),
            component_areas_pixels=component_areas,
            component_count_resolution=[320,240] if self.session else [640,480],
            significant_component_rule='area > 0.2 * largest_component_area; before largest-mask selection',
            mask_png_sha256=sha256_bytes(mask_raw), **geometry, timing_ms=timing,
            model_sha256=self.model_hash, session_evidence=str(self.evidence_path),
            session_evidence_sha256=self.evidence_hash,
            npu_execution_confirmed=self.backend == 'npu' and self.hardware['confirmed'],
            hardware_execution_scope='this persistent session verified before accepting requests',
            foreground_definition='largest red-looking component; not semantic object identity',
            pose_available=False, motion_command_ready=False)


def infer_remote(raw, metadata=None, endpoint='http://127.0.0.1:18820', timeout=2.0, preprocessed=False):
    """Standard-library client; RTT includes HTTP/queue/conversion/model/mask transfer."""
    metadata = dict(metadata or {'source_kind': 'saved_image'})
    if preprocessed and metadata.get('schema') == 'grasp-preprocess-v1':
        expected = {'from_size': [640, 480], 'to_size': [320, 240],
            'interpolation': 'INTER_AREA_2X_EXACT', 'layout': 'HWC', 'channels': 'RGB', 'dtype': 'uint8'}
        if metadata.get('transform') != expected:
            raise ValueError('unsupported FPGA preprocessor transform')
        source_kind = metadata['source_kind']
        metadata['source_kind'] = {'replay': 'saved_image', 'live_wrist': 'live_camera'}.get(source_kind, source_kind)
        metadata['input_format'] = 'rgb_u8_320x240'
        metadata['resize_transform'] = {'source_width': 640, 'source_height': 480,
            'target_width': 320, 'target_height': 240, 'algorithm': 'area_2x2'}
        metadata['preprocessor_evidence'] = metadata.get('preprocessor')
        metadata['producer_timing_ms'] = metadata.pop('timing_ms', {})
        if metadata['source_kind'] == 'live_camera':
            for key in ('camera_serial', 'frame_index', 'board_age_at_fetch_ms', 'received_host_monotonic'):
                metadata[key] = metadata.get('frame', {}).get(key)
    metadata['image_sha256'] = sha256_bytes(raw)
    headers = {'Content-Type': 'application/octet-stream',
               'X-Frame-Metadata': json.dumps(metadata, separators=(',', ':'))}
    started = time.perf_counter()
    request = urllib.request.Request(endpoint.rstrip('/')+('/infer_preprocessed' if preprocessed else '/infer'), data=raw, headers=headers, method='POST')
    with urllib.request.urlopen(request, timeout=timeout) as response:
        payload = response.read()
    transferred = time.perf_counter()
    result = json.loads(payload)
    mask = base64.b64decode(result['mask_png_base64'], validate=True)
    original_hash = metadata.get('original_image_sha256') if preprocessed else metadata['image_sha256']
    if result['image_sha256'] != original_hash or result['submitted_payload_sha256'] != metadata['image_sha256'] or sha256_bytes(mask) != result['mask_png_sha256']:
        raise ValueError('result/image/mask identity mismatch')
    result['timing_ms']['client_http_roundtrip_ms'] = (transferred-started)*1000
    result['timing_ms']['client_parse_validate_ms'] = (time.perf_counter()-transferred)*1000
    result['timing_ms']['client_end_to_end_ms'] = (time.perf_counter()-started)*1000
    return result


def serve(args):
    backend = RedForegroundBackend(args.backend, args.output, args.model)
    lock, slots = threading.Lock(), threading.BoundedSemaphore(2)
    log_lock = threading.Lock()
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'
        def log_message(self, *unused):
            pass
        def reply(self, value, code=200):
            raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        def do_GET(self):
            if self.path != '/health':
                self.reply({'error': 'unknown endpoint'}, 404); return
            self.reply(dict(status='READY', backend=backend.backend, startup_ms=backend.startup_ms,
                model_sha256=backend.model_hash, npu_execution_confirmed=backend.hardware['confirmed'],
                session_evidence=str(backend.evidence_path), requests_completed=backend.sequence))
        def do_POST(self):
            received = time.perf_counter()
            if self.path not in ('/infer', '/infer_preprocessed'):
                self.reply({'error': 'unknown endpoint'}, 404); return
            if not slots.acquire(blocking=False):
                self.reply({'error': 'one active plus one pending request limit; submit freshest frame'}, 503); return
            try:
                length = int(self.headers.get('Content-Length', 0))
                if not 0 < length <= MAX_IMAGE_BYTES:
                    raise ValueError('invalid content length')
                self.connection.settimeout(2.0)
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError('incomplete image payload')
                metadata = json.loads(self.headers.get('X-Frame-Metadata', '{}'))
                queued = time.perf_counter()
                with lock:
                    started = time.perf_counter()
                    result = backend.infer_bytes(raw, metadata, queue_ms=(started-queued)*1000,
                        preprocessed=self.path == '/infer_preprocessed')
                result['timing_ms']['server_request_read_ms'] = (queued-received)*1000
                result['timing_ms']['server_before_response_ms'] = (time.perf_counter()-received)*1000
                row = {k:v for k,v in result.items() if k != 'mask_png_base64'}
                with log_lock:
                    with (backend.evidence_dir/'requests.jsonl').open('a') as handle:
                        handle.write(json.dumps(row, allow_nan=False)+'\n')
                self.reply(result)
            except (ValueError, KeyError, TypeError, TimeoutError) as error:
                self.reply({'error': f'{type(error).__name__}: {error}'}, 400)
            except Exception as error:
                self.reply({'error': f'{type(error).__name__}: {error}'}, 500)
            finally:
                slots.release()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(json.dumps({'status': 'READY', 'port': args.port, 'evidence': str(backend.evidence_path)}), flush=True)
    server.serve_forever(poll_interval=.2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    srv = sub.add_parser('serve')
    srv.add_argument('--backend', choices=('npu', 'cpu', 'hsv'), default='npu')
    srv.add_argument('--model', type=Path, default=MODEL)
    srv.add_argument('--output', type=Path, required=True)
    srv.add_argument('--port', type=int, default=18820)
    infer = sub.add_parser('infer')
    infer.add_argument('--image', type=Path, required=True)
    infer.add_argument('--output', type=Path, required=True)
    infer.add_argument('--endpoint', default='http://127.0.0.1:18820')
    args = parser.parse_args(argv)
    if args.command == 'serve':
        serve(args)
    else:
        result = infer_remote(args.image.read_bytes()) if args.endpoint == 'http://127.0.0.1:18820' else infer_remote(args.image.read_bytes(), endpoint=args.endpoint)
        save_json(args.output, result)
        print(json.dumps({k: result[k] for k in ('status', 'backend', 'roi_xyxy_pixels', 'timing_ms', 'npu_execution_confirmed')}))


if __name__ == '__main__':
    main()
