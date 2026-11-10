#!/usr/bin/env python3
"""Measure received/decoded MJPEG frames, not just camera capture counters."""
import argparse
import json
from pathlib import Path
import statistics
import time
import urllib.request

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:18760')
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', type=Path, default=Path('output/kr260_live_check'))
    args = parser.parse_args()
    if not 2 <= args.seconds <= 120:
        parser.error('seconds must be 2..120')
    args.output.mkdir(parents=True, exist_ok=True)
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def get(path):
        return opener.open(args.url + path, timeout=5)
    with get('/status') as response:
        before = json.load(response)
    if before.get('service') != 'kr260-live-v1' or before.get('error'):
        raise RuntimeError('Live server is not healthy: ' + str(before))
    arrivals, indices, ages, decode_ms = [], [], [], []
    byte_count = 0
    first = None
    start = time.monotonic()
    with get('/color.mjpg') as response:
        while time.monotonic() - start < args.seconds:
            line = response.readline()
            if line != b'--frame\r\n':
                if not line:
                    raise RuntimeError('Stream ended unexpectedly')
                continue
            headers = {}
            while True:
                line = response.readline()
                if not line:
                    raise RuntimeError('Incomplete frame header')
                if line == b'\r\n':
                    break
                key, value = line.decode().strip().split(':', 1)
                headers[key.lower()] = value.strip()
            length = int(headers['content-length'])
            if not 0 < length < 4_000_000:
                raise RuntimeError('Invalid JPEG length')
            raw = response.read(length)
            if len(raw) != length:
                raise RuntimeError('Truncated JPEG')
            now = time.monotonic()
            image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape != (480, 640, 3):
                raise RuntimeError('Invalid decoded color frame')
            decode_ms.append(1000 * (time.monotonic() - now))
            arrivals.append(now)
            indices.append(int(headers['x-frame-index']))
            ages.append(float(headers['x-board-age-ms']))
            byte_count += length
            if first is None:
                first = raw
    stream_duration = time.monotonic() - start
    depth_times, depth_valid = [], []
    for _ in range(10):
        t = time.monotonic()
        with get('/depth.raw') as response:
            w, h = int(response.headers['X-Width']), int(response.headers['X-Height'])
            raw = response.read()
            if len(raw) != w * h * 2:
                raise RuntimeError('Truncated depth frame')
            depth = np.frombuffer(raw, dtype='<u2')
            depth_valid.append(float(((depth > 0) & (depth < 65535)).mean()))
        depth_times.append(1000 * (time.monotonic() - t))
        time.sleep(.1)
    with get('/status') as response:
        after = json.load(response)
    elapsed = arrivals[-1] - arrivals[0]
    def summary(values):
        return dict(median=statistics.median(values), p95=float(np.percentile(values, 95)), max=max(values))
    result = dict(status='completed', duration_s=elapsed, decoded_frames=len(arrivals),
        unique_indices=len(set(indices)), received_fps=(len(arrivals)-1)/elapsed,
        duplicate_frames=len(indices)-len(set(indices)),
        skipped_source_indices=sum(max(0, b-a-1) for a, b in zip(indices, indices[1:])),
        jpeg_payload_mbps=byte_count*8/stream_duration/1e6,
        interarrival_ms=summary([1000*(b-a) for a,b in zip(arrivals,arrivals[1:])]),
        board_frame_age_at_send_ms=summary(ages), jpeg_decode_ms=summary(decode_ms),
        depth_request_total_ms=summary(depth_times), depth_valid_fraction=summary(depth_valid),
        before=before, after=after,
        limitations=['No exposure-to-display latency measurement.',
                      'Color/depth are independent, not aligned or exposure synchronized.',
                      'Depth valid fraction is not absolute distance accuracy.',
                      'This is a receiver decode benchmark, not browser display FPS.'])
    (args.output/'report.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.output/'received_color.jpg').write_bytes(first)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
