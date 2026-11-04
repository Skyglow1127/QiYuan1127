"""Official LeRobot device interfaces for an already linked PiPER pair.

The real leader is the sole motion source. send_action only acknowledges an
observed bus target; arbitrary actions are rejected and no CAN TX API exists.
This Robot is recording-only and cannot deploy a learned policy.
"""
import fcntl
import json
import logging
import os
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
import urllib.request

import cv2
import numpy as np

from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from .transport import PassiveCAN, link_status

WORKSPACE = Path(os.environ.get('LEROBOT_WORKSPACE', '/home/aup/Lerobot'))
IO_CONFIG = Path(__file__).with_name('io_config.json')
PROOF = Path(__file__).with_name('mapping_verification.json')
KEYS = [f'joint_{i+1}.deg' for i in range(6)] + ['gripper.m']
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


@CameraConfig.register_subclass('piper_http')
@dataclass
class PiperHttpCameraConfig(CameraConfig):
    role: str = 'global'
    url: str = 'http://127.0.0.1:18761'
    serial: str = 'CC1365302B2'
    fps: int = 30
    width: int = 640
    height: int = 480
    max_age_ms: int = 150


class PiperHttpCamera(Camera):
    def __init__(self, config):
        super().__init__(config)
        self.config, self.connected, self.last_index = config, False, None
        self.last_metadata = None

    @property
    def is_connected(self): return self.connected

    @staticmethod
    def find_cameras():
        return [{'role': role, 'url': url, 'serial': serial} for role, url, serial in
                [('global', 'http://127.0.0.1:18761', 'CC1365302B2'),
                 ('wrist', 'http://127.0.0.1:18760', 'CC1365301EY')]]

    def connect(self, warmup=True):
        with OPENER.open(self.config.url+'/status', timeout=2) as response:
            status = json.load(response)
        if (status.get('role') != self.config.role or
            status.get('device', {}).get('serial') != self.config.serial or
            status.get('stopped') or status.get('error')):
            raise RuntimeError(f'{self.config.role} 相机身份不符或未就绪')
        self.connected = True
        if warmup: self.read()

    def _fetch(self):
        if not self.connected: raise RuntimeError('相机尚未连接')
        start = time.monotonic()
        with OPENER.open(self.config.url+'/color.jpg', timeout=1) as response:
            data = response.read()
            index = int(response.headers['X-Frame-Index'])
            age = float(response.headers['X-Board-Age-Ms'])
        received = time.monotonic()
        age += (received-start)*1000
        if age > self.config.max_age_ms: raise TimeoutError('相机图像未更新')
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape != (self.height, self.width, 3):
            raise RuntimeError('图像解码或尺寸不符')
        self.last_metadata = {'index': index, 'serial': self.config.serial,
            'receive_monotonic': received, 'estimated_capture_monotonic': received-age/1000,
            'http_rtt_ms': (received-start)*1000, 'age_ms': age}
        return index, cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def read(self): return self.async_read()

    def async_read(self, timeout_ms=200):
        deadline = time.monotonic()+timeout_ms/1000
        while True:
            index, image = self._fetch()
            if self.last_index is None or index > self.last_index:
                self.last_index = index
                return image
            if index < self.last_index: raise RuntimeError('相机帧号倒退，请重连')
            if time.monotonic() >= deadline: raise TimeoutError('相机没有提供新帧')
            time.sleep(.002)

    def read_latest(self, max_age_ms=150):
        index, image = self._fetch()
        if self.last_metadata['age_ms'] > max_age_ms: raise TimeoutError('相机图像过期')
        return image

    def disconnect(self): self.connected = False


def default_cameras():
    return {'global': PiperHttpCameraConfig(),
            'wrist': PiperHttpCameraConfig(role='wrist', url='http://127.0.0.1:18760', serial='CC1365301EY')}


@RobotConfig.register_subclass('piper_shared_observer')
@dataclass
class PiperSharedObserverConfig(RobotConfig):
    io_config: Path = IO_CONFIG
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras)
    data_root: Path | None = None
    connect_timeout_s: float = 60.
    calibration_dir: Path = WORKSPACE/'calibration/piper_shared_observer'


@TeleoperatorConfig.register_subclass('piper_shared_leader')
@dataclass
class PiperSharedLeaderConfig(TeleoperatorConfig):
    io_config: Path = IO_CONFIG
    connect_timeout_s: float = 60.
    calibration_dir: Path = WORKSPACE/'calibration/piper_shared_leader'


def interface_identity(interface):
    net = Path('/sys/class/net')/interface
    return {'interface': interface, 'ifindex': int((net/'ifindex').read_text()),
            'device_path': str((net/'device').resolve())}


def inspect_endpoint(config):
    transport = config.get('transport', {})
    if transport.get('kind', 'local') == 'local':
        return {'interface_identity': interface_identity(config['interface']),
                'link': link_status(config['interface'])}
    if transport['kind'] != 'ssh':
        raise RuntimeError('不支持的 CAN 接收方式')
    from .ssh_transport import remote_probe
    message = remote_probe(config)
    if message['identity'] != transport.get('expected_identity'):
        raise RuntimeError('KR260 CAN 接收端身份与绑定不符，请重新核验')
    return {'interface_identity': {'transport': 'ssh',
                'ssh_host': transport['ssh_host'],
                'ssh_hostname': transport.get('ssh_hostname'),
                'receiver': message['identity']}, 'link': message['link']}


def proof_path(config, config_path=IO_CONFIG):
    name = config.get('mapping_proof')
    if not name:
        return PROOF
    path = Path(name)
    return path if path.is_absolute() else Path(config_path).parent/path


def check_endpoint(config, config_path=IO_CONFIG):
    endpoint = inspect_endpoint(config)
    if not endpoint['link']['ready']:
        location = 'KR260' if config.get('transport', {}).get('kind') == 'ssh' else '本机'
        raise RuntimeError(location+' can0 需要 UP + LISTEN-ONLY + 1 Mbps；请在对应终端启用监听')
    path = proof_path(config, config_path)
    if not path.exists():
        raise RuntimeError('尚未完成当前接线的主臂目标核验，请先运行 verify_kr260_can.py')
    proof = json.loads(path.read_text())
    if (proof['interface_identity'] != endpoint['interface_identity'] or
        proof['mapping'] != config['mapping'] or proof.get('joint_unit') != 'deg' or
        proof['gripper_mode'] != 'width'):
        raise RuntimeError('PiPER接口身份/映射与已核验证据不符，请先核验硬件接线')
    if (config.get('transport', {}).get('kind') == 'ssh' and
        proof.get('current_source_verified') is not True):
        raise RuntimeError('KR260 当前完整主臂目标尚未核验')
    return endpoint


class SharedContext:
    """Shared RX session, not a recording loop. Official code requests each sample."""
    def __init__(self, config_path):
        self.config_path = Path(config_path)
        self.config = json.loads(self.config_path.read_text())
        self.bus, self.refs, self.data_root = None, 0, None
        self.cycle = None
        self.expected_action = None
        self.sample_number = 0
        self.raw_file = self.sample_file = self.lock_file = None

    def acquire(self, timeout):
        if self.refs:
            self.refs += 1
            return
        endpoint = check_endpoint(self.config, self.config_path)
        lock_path = WORKSPACE/'logs/piper_official_record.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        self.lock_file = lock_path.open('a')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.bus = PassiveCAN(self.config)
            log_dir = ((self.data_root/'meta/piper') if self.data_root else
                       WORKSPACE/'logs/official_record'/str(time.time_ns()))
            log_dir.mkdir(parents=True, exist_ok=True)
            suffix = str(time.time_ns())
            self.raw_file = (log_dir/f'can_{suffix}.jsonl').open('w', buffering=65536)
            self.sample_file = (log_dir/f'samples_{suffix}.jsonl').open('w', buffering=65536)
            self.bus.attach_raw(self.raw_file)
            (log_dir/f'session_{suffix}.json').write_text(json.dumps({
                'recording_engine': 'lerobot.scripts.lerobot_record', 'adapter': 'piper_shared_observer',
                'units': self.config['canonical_units'], 'io_config': self.config,
                'endpoint': endpoint, 'mapping_proof': str(proof_path(self.config, self.config_path)),
                'motion_owner': 'existing physical leader-follower link', 'robot_commands_sent': 0,
                'raw_sample_scope': 'all official record/reset cycles; conservative host times plus remote source times when using SSH',
                'camera_timing': 'HTTP round-trip age bound, no exposure synchronization'}, indent=2)+'\n')
            logging.warning('PiPER只监听：等待主臂真实目标；请准备起始姿态。连接完成后官方录制会自动开始。')
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                snapshot = self.bus.snapshot()
                fresh = all(snapshot['ages_ms'].get(f'action_{i}', 1e9) <= self.config['max_can_age_ms']
                            for i in range(4))
                if snapshot['ready'] and fresh:
                    self.refs = 1
                    return
                time.sleep(.005)
            raise TimeoutError('未收到完整新目标与实时执行臂反馈。请检查供电、随动及CAN连接后重试。')
        except BaseException:
            self._close()
            raise

    def release(self):
        self.refs = max(0, self.refs-1)
        if not self.refs: self._close()

    def _close(self):
        if self.bus:
            self.bus.close()
            self.bus = None
        for name in ('raw_file', 'sample_file', 'lock_file'):
            handle = getattr(self, name)
            if handle:
                handle.close()
                setattr(self, name, None)
        if self.data_root: self.write_manifest()

    @staticmethod
    def _can_wait_for_fresh(snapshot):
        """Only stale, otherwise complete and healthy feedback may recover."""
        allowed = {f'{kind} 第 {part} 组数据过期'
                   for kind in ('state', 'action') for part in range(1, 5)}
        allowed.add('执行臂状态反馈缺失或过期')
        status = snapshot.get('arm_status')
        return bool(
            snapshot.get('issues') and set(snapshot['issues']) <= allowed and
            not snapshot.get('error') and not snapshot.get('raw_error') and
            snapshot.get('link', {}).get('ready') and
            all(snapshot.get(kind) is not None and len(snapshot[kind]) == 7
                for kind in ('state', 'action')) and
            status and status.get('arm_status') == 0 and status.get('err_code') == 0 and
            status.get('ctrl_mode') == 1 and status.get('mode_feedback') == 1)

    def checked_snapshot(self, wait_for_fresh_s=0, *, phase='snapshot'):
        if not self.bus: raise RuntimeError('CAN监听尚未连接')
        started = time.monotonic()
        deadline = started + max(0., wait_for_fresh_s)
        while True:
            snapshot = self.bus.snapshot()
            if snapshot['ready']:
                return snapshot
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not self._can_wait_for_fresh(snapshot):
                break
            time.sleep(min(.005, remaining))
        sample_file = getattr(self, 'sample_file', None)
        if sample_file is not None:
            try:
                path = Path(sample_file.name).parent/f'can_feedback_error_{time.time_ns()}.json'
                path.write_text(json.dumps({
                    'snapshot': snapshot, 'waited_s': time.monotonic()-started,
                    'wait_for_fresh_s': wait_for_fresh_s, 'sample_number': self.sample_number,
                    'phase': phase,
                }, ensure_ascii=False, indent=2)+'\n')
            except Exception:
                logging.warning('无法写入 CAN 反馈失败诊断', exc_info=True)
        raise RuntimeError('；'.join(snapshot['issues']))

    def acknowledge(self, action):
        if self.expected_action is None or set(action) != set(KEYS):
            raise RuntimeError('录制专用适配器只接受本帧已监听到的主臂目标')
        if not np.allclose([action[k] for k in KEYS], [self.expected_action[k] for k in KEYS], atol=1e-6, rtol=0):
            raise RuntimeError('动作被修改；此只监听适配器不能执行模型或任意目标')
        try:
            self.checked_snapshot(phase='acknowledge')
        except RuntimeError:
            self.cycle = self.expected_action = None
            raise
        if self.sample_file:
            self.sample_file.write(json.dumps({'cycle': self.sample_number,
                'host_monotonic': time.monotonic(), 'can': self.cycle,
                'action': action}, separators=(',', ':'))+'\n')
        self.sample_number += 1
        return dict(action)

    def write_manifest(self):
        info_path = self.data_root/'meta/info.json'
        if not info_path.exists(): return
        info = json.loads(info_path.read_text())
        count = info['total_episodes']
        plan = json.loads((WORKSPACE/'projects/piper/collection_plan.json').read_text())
        validation = [i for i in range(min(count, 50)) if plan['planned_slots'][i]['suggested_split']=='validation']
        manifest = {'recording_engine': 'official lerobot-record', 'units': {'joints':'deg','gripper':'m'},
            'train_episode_indices': [i for i in range(count) if i not in validation],
            'validation_episode_indices': validation,
            'episode_quality': 'Operator must review success; official r key discards/re-records. No automatic grasp-success detection.',
            'position_groups': 'Follow collection_plan.json manually; no automatic position verification.'}
        (self.data_root/'collection_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')


_contexts = {}
_context_lock = threading.Lock()


def context_for(path):
    key = str(Path(path).resolve())
    with _context_lock:
        if key not in _contexts: _contexts[key] = SharedContext(key)
        return _contexts[key]


class PiperSharedObserver(Robot):
    config_class = PiperSharedObserverConfig
    name = 'piper_shared_observer'

    def __init__(self, config):
        super().__init__(config)
        self.config, self.connected = config, False
        self.context = context_for(config.io_config)
        self.context.data_root = Path(config.data_root) if config.data_root else None
        self.cameras = {name: PiperHttpCamera(camera) for name, camera in config.cameras.items()}
        self._camera_executor = None
        self._last_pair_timing = None

    @property
    def observation_features(self):
        return {**self.action_features, **{name: (camera.height, camera.width, 3) for name, camera in self.cameras.items()}}

    @property
    def action_features(self): return dict.fromkeys(KEYS, float)
    @property
    def is_connected(self): return self.connected
    @property
    def is_calibrated(self): return True  # Absolute feedback; no writes or zeroing.
    def calibrate(self): pass
    def configure(self): pass

    def connect(self, calibrate=True):
        if self.connected: raise RuntimeError('已连接')
        self.context.acquire(self.config.connect_timeout_s)
        try:
            for camera in self.cameras.values(): camera.connect()
            self.connected = True
        except BaseException:
            for camera in self.cameras.values(): camera.disconnect()
            self.context.release()
            raise

    def get_observation(self):
        if not self.connected: raise RuntimeError('执行臂监听尚未连接')
        # A failed acquisition must not expose the preceding cycle's action.
        self.context.cycle = self.context.expected_action = None
        started = time.monotonic()
        deadline = started + .4
        self.context.checked_snapshot(wait_for_fresh_s=min(.15, max(0, deadline-time.monotonic())),
                                      phase='before_cameras')
        for attempt in range(1, 3):
            images = self._read_images(deadline=deadline)
            # Wait only for actual fresh CAN, then age the images again: CAN
            # recovery can consume the camera pair's remaining freshness.
            snapshot = self.context.checked_snapshot(
                wait_for_fresh_s=min(.15, max(0, deadline-time.monotonic())), phase='after_cameras')
            now = time.monotonic()
            ages = {name: (now-camera.last_metadata['estimated_capture_monotonic'])*1000
                    for name, camera in self.cameras.items()}
            self._last_pair_timing.update(ages_ms=ages, observation_attempts=attempt,
                                         observation_ms=(now-started)*1000)
            if all(0 <= age <= min(self.context.config['max_camera_age_ms'],
                       self.cameras[name].config.max_age_ms) for name, age in ages.items()):
                for name, age in ages.items():
                    self.cameras[name].last_metadata['sample_age_ms'] = age
                self.context.cycle = {'sampled_monotonic': now, 'snapshot': snapshot,
                    'cameras': {name: dict(camera.last_metadata) for name, camera in self.cameras.items()},
                    'camera_pair': dict(self._last_pair_timing)}
                self.context.expected_action = dict(zip(KEYS, snapshot['action'], strict=True))
                return {**dict(zip(KEYS, snapshot['state'], strict=True)), **images}
            if attempt == 2 or now >= deadline:
                break
        details = dict(self._last_pair_timing,
                       frames={name: camera.last_metadata for name, camera in self.cameras.items()})
        self._save_camera_pair_error(details)
        raise RuntimeError('等待CAN新反馈后双相机帧仍超过150ms年龄门限：'+
                           json.dumps(details, ensure_ascii=False))

    def _read_images(self, *, deadline=None):
        """Read concurrently and refresh older frames within a bounded window.

        The original conservative HTTP age estimate and 150/60 ms gates stay in
        effect. This pairs received frames; it does not synchronize exposures.
        """
        if self._camera_executor is None:
            self._camera_executor = ThreadPoolExecutor(max_workers=max(1, len(self.cameras)),
                                                       thread_name_prefix='piper-camera')
        started = time.monotonic()
        # The observation supplies a shared 400ms budget, including any CAN
        # recovery. Do not cut that remaining budget to 200ms: one preempted
        # HTTP fetch can consume 200ms yet leave time to acquire fresh frames.
        # Standalone pairing retains its existing 200ms default. Accepted
        # frame age/skew limits are unchanged; long reads remain observable.
        deadline = min(started+.4, deadline) if deadline is not None else started+.2
        images, metadata = {}, {}
        pending = set(self.cameras)
        last_timeout = None
        for attempt in range(1, 5):
            remaining_ms = max(1, int((deadline-time.monotonic())*1000))
            futures = {name: self._camera_executor.submit(self.cameras[name].async_read,
                       timeout_ms=remaining_ms) for name in self.cameras if name in pending}
            failed = set()
            fatal = None
            # Drain every submitted future before retry/disconnect so no camera
            # has two concurrent readers modifying its last frame metadata.
            for name, future in futures.items():
                try:
                    images[name] = future.result()
                    metadata[name] = dict(self.cameras[name].last_metadata)
                except TimeoutError as error:
                    failed.add(name)
                    last_timeout = str(error)
                except Exception as error:
                    fatal = error
            if fatal is not None:
                raise fatal
            now = time.monotonic()
            ages = {name: (now-item['estimated_capture_monotonic'])*1000
                    for name, item in metadata.items()}
            times = [item['estimated_capture_monotonic'] for item in metadata.values()]
            skew = (max(times)-min(times))*1000 if times else None
            self._last_pair_timing = {'attempts': attempt, 'skew_ms': skew,
                'ages_ms': ages, 'acquisition_ms': (now-started)*1000}
            if (not failed and len(images) == len(self.cameras) and
                all(0 <= age <= min(self.context.config['max_camera_age_ms'],
                    self.cameras[name].config.max_age_ms) for name, age in ages.items()) and
                (skew is None or skew <= self.context.config['max_camera_skew_ms'])):
                for name in self.cameras:
                    metadata[name]['sample_age_ms'] = ages[name]
                    self.cameras[name].last_metadata = metadata[name]
                return images
            if attempt == 4 or now >= deadline:
                break
            newest = max(times) if times else now
            pending = failed | (set(self.cameras)-set(images)) | {
                name for name, item in metadata.items()
                if ages[name] < 0 or
                ages[name] > min(self.context.config['max_camera_age_ms'],
                                 self.cameras[name].config.max_age_ms) or
                (newest-item['estimated_capture_monotonic'])*1000 > self.context.config['max_camera_skew_ms']}
        details = dict(self._last_pair_timing or {}, last_timeout=last_timeout,
                       frames=metadata)
        # Preserve rejected-pair evidence too; failed observations do not reach
        # the ordinary sample log and previously left no useful timing record.
        self._save_camera_pair_error(details)
        raise RuntimeError('双相机在有限重配对后仍未满足60ms时差/150ms帧年龄：'+
                           json.dumps(details, ensure_ascii=False))

    def _save_camera_pair_error(self, details):
        sample_file = getattr(self.context, 'sample_file', None)
        if sample_file is not None:
            path = Path(sample_file.name).parent/f'camera_pair_error_{time.time_ns()}.json'
            try:
                path.write_text(json.dumps(details, ensure_ascii=False, indent=2)+'\n')
            except OSError as error:
                logging.warning('相机失败诊断写入失败：%s', error)

    def send_action(self, action):
        # The official loop calls this method; no control message is sent.
        return self.context.acknowledge(action)

    def disconnect(self):
        if not self.connected: return
        self.connected = False
        if self._camera_executor is not None:
            self._camera_executor.shutdown(wait=True, cancel_futures=True)
            self._camera_executor = None
        for camera in self.cameras.values(): camera.disconnect()
        self.context.release()


class PiperSharedLeader(Teleoperator):
    config_class = PiperSharedLeaderConfig
    name = 'piper_shared_leader'

    def __init__(self, config):
        super().__init__(config)
        self.config, self.connected = config, False
        self.context = context_for(config.io_config)

    @property
    def action_features(self): return dict.fromkeys(KEYS, float)
    @property
    def feedback_features(self): return {}
    @property
    def is_connected(self): return self.connected
    @property
    def is_calibrated(self): return True
    def calibrate(self): pass
    def configure(self): pass

    def connect(self, calibrate=True):
        if self.connected: raise RuntimeError('已连接')
        self.context.acquire(self.config.connect_timeout_s)
        self.connected = True

    def get_action(self):
        if not self.connected or self.context.expected_action is None:
            raise RuntimeError('等待本帧的执行臂观测和主臂目标')
        return dict(self.context.expected_action)

    def send_feedback(self, feedback):
        if feedback: raise RuntimeError('此适配器不向主臂发送任何反馈指令')

    def disconnect(self):
        if not self.connected: return
        self.connected = False
        self.context.release()
