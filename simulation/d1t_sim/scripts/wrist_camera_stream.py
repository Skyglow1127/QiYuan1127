"""Latest real Gazebo RGB camera frame, encoded off the control/HTTP threads.

Gazebo msgs10 image.proto defines RGB_INT8=3 and `step` as bytes per row.
Only that RGB8 format is accepted; no synthetic or cached-online fallback.
"""
import io
import threading
import time

from gz.msgs10.image_pb2 import Image as GzImage, PixelFormatType, RGB_INT8
from gz.transport13 import Node as GzNode
from PIL import Image


class WristCameraStream:
    TOPIC = '/d1t/wrist_camera/image'
    MAX_AGE = 2.0

    def __init__(self):
        self._condition = threading.Condition()
        self._closed = False
        self._pending = None
        self._jpeg = None
        self._last = 0.0
        self._generation = 0
        self._frames = 0
        self._received = 0
        self._dropped = 0
        self._metadata = {'width': 0, 'height': 0, 'step': 0,
                          'pixel_format': None, 'pixel_format_type': None,
                          'sim_time': None, 'frame_id': 0}
        self._error = '等待 Gazebo 腕部相机帧'
        self._worker = threading.Thread(target=self._encode, name='wrist-camera-jpeg', daemon=True)
        self._worker.start()
        self.gz = GzNode()
        # subscribe() deserializes protobuf on the transport callback thread.
        if self.gz.subscribe(GzImage, self.TOPIC, self._on_image) is False:
            with self._condition:
                self._error = '无法订阅 Gazebo 腕部相机主题'

    def _on_image(self, message):
        received = time.monotonic()
        width, height, step = int(message.width), int(message.height), int(message.step)
        pixel_format = int(message.pixel_format_type)
        with self._condition:
            if self._closed:
                return
            self._received += 1
            self._generation += 1
            generation = self._generation
            try:
                format_name = PixelFormatType.Name(pixel_format)
            except ValueError:
                format_name = f'UNKNOWN({pixel_format})'
            error = None
            if pixel_format != RGB_INT8:
                error = f'腕部相机像素格式不支持：{format_name}，要求 RGB_INT8'
            elif not (0 < width <= 4096 and 0 < height <= 4096):
                error = '腕部相机图像尺寸无效'
            elif step < width * 3 or len(message.data) != step * height:
                error = '腕部相机 RGB8 数据长度或行跨度无效'
            if error:
                self._jpeg = None
                self._pending = None
                self._error = error
                return
            if self._pending is not None:
                self._dropped += 1
            metadata = {'width': width, 'height': height, 'step': step,
                        'pixel_format': format_name, 'pixel_format_type': pixel_format,
                        'sim_time': message.header.stamp.sec + message.header.stamp.nsec * 1e-9,
                        'frame_id': generation}
            # Keep only the newest raw frame; encoding never blocks the ROS timer.
            self._pending = (generation, received, bytes(message.data), metadata)
            self._condition.notify()

    def _encode(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                generation, received, data, metadata = self._pending
                self._pending = None
            try:
                image = Image.frombytes('RGB', (metadata['width'], metadata['height']), data,
                                        'raw', 'RGB', metadata['step'], 1)
                output = io.BytesIO()
                image.save(output, format='JPEG', quality=85)
                jpeg = output.getvalue()
            except (OSError, ValueError) as exc:
                with self._condition:
                    if generation == self._generation:
                        self._jpeg = None
                        self._error = '腕部相机 JPEG 编码失败：' + str(exc)
                continue
            with self._condition:
                if self._closed:
                    return
                # A malformed/newer frame invalidates an image still being encoded.
                if generation != self._generation:
                    self._dropped += 1
                    continue
                self._jpeg = jpeg
                self._metadata = metadata
                self._last = received
                self._frames += 1
                self._error = None

    def snapshot(self):
        with self._condition:
            age = None if not self._last else max(0.0, time.monotonic() - self._last)
            connected = bool(self._jpeg is not None and not self._closed
                             and age is not None and age <= self.MAX_AGE)
            error = self._error
            if not connected and error is None:
                error = 'Gazebo 腕部相机帧已超时'
            state = {'connected': connected, 'age': age, 'topic': self.TOPIC,
                     'source': 'Gazebo RGB camera sensor', 'encoding': 'image/jpeg',
                     'frames': self._frames, 'received_frames': self._received,
                     'dropped_frames': self._dropped, 'max_age': self.MAX_AGE,
                     'error': error, **self._metadata}
            return (self._jpeg if connected else None), state

    def state(self):
        return self.snapshot()[1]

    def close(self):
        with self._condition:
            self._closed = True
            self._jpeg = None
            self._pending = None
            self._condition.notify_all()
        self.gz.unsubscribe(self.TOPIC)
        self._worker.join(timeout=1)
