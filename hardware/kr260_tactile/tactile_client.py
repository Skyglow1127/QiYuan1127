#!/usr/bin/env python3
"""Read-only integration example for an existing grasp controller; never moves a robot."""
import argparse
import json
import time
from urllib.request import urlopen
from tactile import integration_view


class TactileClient:
    def __init__(self, url='http://127.0.0.1:8765/api/tactile', timeout=.15):
        self.url, self.timeout = url, timeout
        self.last_session = None
        self.last_seq = None

    def read(self):
        """Call for each control decision. HOLD on failure; no cached fallback."""
        started = time.monotonic()
        try:
            with urlopen(self.url, timeout=self.timeout) as response:
                body = response.read(65537)
            if len(body) > 65536:
                raise ValueError('响应过长')
            payload = json.loads(body)
            elapsed_ms = (time.monotonic() - started) * 1000
            if not isinstance(payload, dict):
                raise ValueError('无效响应')
            decision = integration_view(payload)
            if decision['decision'] != 'HOLD':
                seq, session = payload.get('seq'), payload.get('session_id')
                if type(seq) is not int or seq < 1 or not isinstance(session, str) or not session:
                    raise ValueError('缺少有效会话或序号')
                if payload['receive_age_ms'] + elapsed_ms > payload['stale_ms']:
                    raise ValueError('响应到达时已经过期')
                if self.last_session == session and self.last_seq is not None and seq <= self.last_seq:
                    raise ValueError('没有更新的样本；不要重复使用旧触觉值')
                self.last_session, self.last_seq = session, seq
            return dict(**decision, session_id=payload.get('session_id'), seq=payload.get('seq'),
                        channels=payload.get('channels', []), request_ms=elapsed_ms,
                        motion_executed=False)
        except Exception as exc:
            return dict(decision='HOLD', stop_closing=True, both_contact=None,
                        reason=str(exc), motion_executed=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8765/api/tactile')
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    client = TactileClient(args.url)
    end = time.monotonic() + max(0, args.seconds)
    while True:
        print(json.dumps(client.read(), ensure_ascii=False), flush=True)
        if args.once or time.monotonic() >= end:
            break
        time.sleep(.05)


if __name__ == '__main__': main()
