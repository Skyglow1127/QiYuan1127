"""VLA-only bounded camera acquisition; no model, action or guard changes.

Failed frames are discarded. Accepted images keep the existing conservative
header-age plus full HTTP round-trip source estimate, including decode age.
The shared PiPER adapter and the official LeRobot control loop stay unchanged.
"""
from contextlib import contextmanager
from concurrent.futures import ThreadPoolExecutor
import json
import logging
import math
import time
from urllib.error import URLError

import cv2
import numpy as np

from lerobot_robot_piper_executor import backend as original_backend
from lerobot_robot_piper_executor.backend import PiperBackend
from lerobot_robot_piper_executor.guard import ActionGuard, GuardError
from lerobot_robot_piper_executor.robot import PiperPolicyExecutor
from lerobot_robot_piper_shared.devices import OPENER, PiperHttpCamera, PiperHttpCameraConfig

LOG = logging.getLogger(__name__)
PAIR_BUDGET_S = .200
PAIR_ATTEMPTS = 4
MAX_CAMERA_AGE_MS = 150.
MAX_CAMERA_SKEW_MS = 60.


def retryable_timeout(error):
    """HTTP503, transport resets, identity errors and malformed images are fatal."""
    return (isinstance(error, TimeoutError) or
            isinstance(error, URLError) and isinstance(error.reason, TimeoutError))


class VLABoundedHttpCamera(PiperHttpCamera):
    def __init__(self, config, *, clock=time.monotonic, sleep=time.sleep):
        super().__init__(config)
        self._clock, self._sleep = clock, sleep
        self._accepted_generation = None
        self.last_fetch_diagnostic = None

    def _remaining(self, deadline):
        remaining = deadline-self._clock()
        if remaining <= 0:
            raise TimeoutError('VLA camera acquisition deadline exhausted')
        return remaining

    def _fetch_candidate(self, deadline):
        """Return a fully checked candidate without committing accepted metadata."""
        if not self.connected:
            raise RuntimeError('Camera is not connected')
        started = self._clock()
        diagnostic = {'role': self.config.role, 'serial': self.config.serial,
            'last_index': self.last_index, 'index': None, 'board_age_ms': None,
            'http_rtt_ms': None, 'estimated_age_ms': None,
            'remaining_budget_ms': max(0., deadline-started)*1000}
        try:
            with OPENER.open(self.config.url+'/color.jpg', timeout=self._remaining(deadline)) as response:
                self._remaining(deadline)
                headers = response.headers
                index = int(headers['X-Frame-Index'])
                board_age = float(headers['X-Board-Age-Ms'])
                generation = int(headers['X-Camera-Generation'])
                diagnostic.update(index=index, board_age_ms=board_age, generation=generation)
                if (headers['X-Camera-Role'] != self.config.role or
                        headers['X-Camera-Serial'] != self.config.serial):
                    raise RuntimeError('Camera HTTP frame role/serial mismatch')
                if (index < 0 or generation < 0 or not math.isfinite(board_age) or board_age < 0):
                    raise RuntimeError('Camera HTTP frame identity/age is invalid')
                if self._accepted_generation is not None and generation != self._accepted_generation:
                    raise RuntimeError('Camera generation changed; reconnect and review required')
                if self.last_index is not None and index < self.last_index:
                    raise RuntimeError('Camera frame index moved backwards')
                length = int(headers['Content-Length'])
                if length <= 0:
                    raise RuntimeError('Camera HTTP frame has invalid Content-Length')
                # Socket timeouts apply per blocking operation, so reset the
                # remaining absolute budget before every read1, not read-all.
                sock = getattr(getattr(getattr(response, 'fp', None), 'raw', None), '_sock', None)
                if sock is None:
                    raise RuntimeError('Cannot enforce camera HTTP body socket deadline')
                chunks, received_bytes = [], 0
                while received_bytes < length:
                    sock.settimeout(self._remaining(deadline))
                    chunk = response.read1(min(65536, length-received_bytes))
                    self._remaining(deadline)
                    if not chunk:
                        raise RuntimeError('Camera HTTP frame body was truncated')
                    chunks.append(chunk)
                    received_bytes += len(chunk)
                data = b''.join(chunks)
            received = self._clock()
            http_rtt_ms = (received-started)*1000
            estimated_age_ms = board_age+http_rtt_ms
            source_at = received-estimated_age_ms/1000
            diagnostic.update(http_rtt_ms=http_rtt_ms, estimated_age_ms=estimated_age_ms)
            if estimated_age_ms > min(MAX_CAMERA_AGE_MS, self.config.max_age_ms):
                raise TimeoutError('Camera image exceeds unchanged150ms freshness limit')
            image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape != (self.height, self.width, 3):
                raise RuntimeError('Camera image decode/shape mismatch')
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self._remaining(deadline)
            final_age_ms = (self._clock()-source_at)*1000
            diagnostic['age_after_decode_ms'] = final_age_ms
            if not 0 <= final_age_ms <= min(MAX_CAMERA_AGE_MS, self.config.max_age_ms):
                raise TimeoutError('Camera image became stale during decode')
            metadata = {'index': index, 'serial': self.config.serial, 'generation': generation,
                'receive_monotonic': received, 'estimated_capture_monotonic': source_at,
                'http_rtt_ms': http_rtt_ms, 'board_age_ms': board_age,
                'age_ms': estimated_age_ms, 'age_after_decode_ms': final_age_ms}
            return index, image, metadata
        except BaseException as error:
            diagnostic.update(error=type(error).__name__+': '+str(error),
                              http_rtt_ms=(self._clock()-started)*1000 if diagnostic['http_rtt_ms'] is None else diagnostic['http_rtt_ms'],
                              remaining_budget_ms=max(0., deadline-self._clock())*1000)
            if diagnostic['estimated_age_ms'] is None and diagnostic['board_age_ms'] is not None:
                diagnostic['estimated_age_ms'] = diagnostic['board_age_ms']+diagnostic['http_rtt_ms']
            LOG.warning('VLA camera rejected fetch: %s', json.dumps(diagnostic, ensure_ascii=False))
            raise
        finally:
            diagnostic['remaining_budget_ms'] = max(0., deadline-self._clock())*1000
            self.last_fetch_diagnostic = diagnostic

    def async_read(self, timeout_ms=200, *, deadline=None):
        started = self._clock()
        local_deadline = started+timeout_ms/1000
        deadline = local_deadline if deadline is None else min(deadline, local_deadline)
        while True:
            index, image, metadata = self._fetch_candidate(deadline)
            self._remaining(deadline)
            if self.last_index is None or index > self.last_index:
                # No failed, duplicate or post-deadline fetch commits metadata.
                self.last_index, self.last_metadata = index, metadata
                self._accepted_generation = metadata['generation']
                return image
            self._sleep(min(.002, self._remaining(deadline)))

    def _fetch(self):
        """Preserve the parent's read_latest interface without refreshing old frames."""
        deadline = self._clock()+PAIR_BUDGET_S
        index, image, metadata = self._fetch_candidate(deadline)
        self._remaining(deadline)
        if self.last_index is None or index > self.last_index:
            self.last_index, self.last_metadata = index, metadata
            self._accepted_generation = metadata['generation']
        return index, image

    def read_latest(self, max_age_ms=150):
        _, image = self._fetch()
        # A repeated frame keeps its earlier source estimate; never judge its
        # current age from a stored age-at-receive value.
        age = (self._clock()-self.last_metadata['estimated_capture_monotonic'])*1000
        if not 0 <= age <= min(max_age_ms, self.config.max_age_ms, MAX_CAMERA_AGE_MS):
            raise TimeoutError('Camera latest frame exceeds requested freshness limit')
        return image


class VLACameraRecoveryBackend(PiperBackend):
    def __init__(self, config, io, *, client=None, clock=time.monotonic):
        super().__init__(config, io, client=client)
        self._camera_clock = clock
        self.last_camera_pair_timing = None

    def open_read_only(self):
        """Same RPC/codec/cleanup path, with only the VLA camera implementation."""
        try:
            if self.execution_enabled:
                self.target_codec_preparation = original_backend.prepare_target_codec()
            self.client.open()
            for role in ('global', 'wrist'):
                camera = VLABoundedHttpCamera(PiperHttpCameraConfig(role=role,
                    url=self.io['cameras'][role]['url'], serial=self.io['cameras'][role]['serial']),
                    clock=self._camera_clock)
                self.cameras[role] = camera
                camera.connect(warmup=False)
            self.pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix='pi05-vla-camera')
            self._read_camera_pair()
            self.connected = True
        except BaseException:
            self.close()
            raise

    def _read_camera_pair(self):
        started = self._camera_clock()
        deadline = started+PAIR_BUDGET_S
        images, metadata = {}, {}
        pending = set(self.cameras)
        last_errors = {}
        for attempt in range(1, PAIR_ATTEMPTS+1):
            remaining = deadline-self._camera_clock()
            if remaining <= 0:
                break
            futures = {role: self.pool.submit(camera.async_read,
                       timeout_ms=remaining*1000, deadline=deadline)
                       for role, camera in self.cameras.items() if role in pending}
            failed, fatal = set(), None
            # Always drain every submitted future before retry or close. Never
            # start another reader for a camera whose previous read is active.
            for role, future in futures.items():
                try:
                    images[role] = future.result()
                    metadata[role] = dict(self.cameras[role].last_metadata)
                except BaseException as error:
                    images.pop(role, None)
                    metadata.pop(role, None)
                    last_errors[role] = type(error).__name__+': '+str(error)
                    if retryable_timeout(error):
                        failed.add(role)
                    elif fatal is None:
                        fatal = error
            now = self._camera_clock()
            ages = {role: (now-item['estimated_capture_monotonic'])*1000
                    for role, item in metadata.items()}
            times = [item['estimated_capture_monotonic'] for item in metadata.values()]
            skew = (max(times)-min(times))*1000 if times else None
            details = {'attempts': attempt, 'acquisition_ms': (now-started)*1000,
                'budget_ms': PAIR_BUDGET_S*1000, 'remaining_budget_ms': max(0., deadline-now)*1000,
                'ages_ms': ages, 'skew_ms': skew, 'pending_roles': sorted(pending),
                'errors': dict(last_errors), 'frames': dict(metadata),
                'fetches': {role: camera.last_fetch_diagnostic for role, camera in self.cameras.items()}}
            self.last_camera_pair_timing = details
            if fatal is not None:
                LOG.error('VLA camera pair fatal: %s', json.dumps(details, ensure_ascii=False))
                raise fatal
            if any(not math.isfinite(age) or age < 0 for age in ages.values()):
                raise GuardError('Camera pair has invalid/future source timestamps')
            age_limits = {role: min(MAX_CAMERA_AGE_MS, self.io['max_camera_age_ms'], camera.config.max_age_ms)
                          for role, camera in self.cameras.items()}
            skew_limit = min(MAX_CAMERA_SKEW_MS, self.io['max_camera_skew_ms'])
            if (now < deadline and not failed and len(images) == len(self.cameras) and
                    all(age <= age_limits[role] for role, age in ages.items()) and
                    skew is not None and skew <= skew_limit):
                for role, camera in self.cameras.items():
                    camera.last_metadata['sample_age_ms'] = ages[role]
                if attempt > 1:
                    LOG.info('VLA camera pair recovered using genuinely new frames: %s',
                             json.dumps(details, ensure_ascii=False))
                return images, min(times)
            if attempt == PAIR_ATTEMPTS or now >= deadline:
                break
            newest = max(times) if times else now
            pending = failed | (set(self.cameras)-set(images)) | {
                role for role, item in metadata.items()
                if ages[role] > age_limits[role] or
                (newest-item['estimated_capture_monotonic'])*1000 > skew_limit}
        details = self.last_camera_pair_timing or {'budget_ms': PAIR_BUDGET_S*1000}
        LOG.error('VLA camera pair deadline/retry budget exhausted: %s', json.dumps(details, ensure_ascii=False))
        raise GuardError('VLA camera pair failed within unchanged150ms age/60ms skew gates: '+
                         json.dumps(details, ensure_ascii=False))


@contextmanager
def vla_camera_backend_injection(context_module):
    """Wrap the already tracked factory before official context.connect()."""
    previous_factory = context_module.make_robot_from_config

    def factory(config):
        robot = previous_factory(config)
        if not isinstance(robot, PiperPolicyExecutor) or robot.is_connected or robot.backend is not None or robot.guard is not None:
            raise RuntimeError('VLA camera injection requires a fresh tracked Piper executor')
        backend = VLACameraRecoveryBackend.from_file(config.deployment_config)
        robot.backend = backend
        # Original connect constructs these together only when backend is None.
        # Preserve exactly the same limits when supplying an explicit backend.
        robot.guard = ActionGuard(backend.guard_limits) if backend.guard_limits is not None else None
        return robot

    context_module.make_robot_from_config = factory
    try:
        yield
    finally:
        context_module.make_robot_from_config = previous_factory
