#!/usr/bin/env python3
"""Read-only preview. Recording exists only in the official lerobot-record CLI."""
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import urllib.request

from lerobot_robot_piper_shared.devices import IO_CONFIG, WORKSPACE
from lerobot_robot_piper_shared.transport import PassiveCAN

PAGE = Path(__file__).with_name('preview_official.html')
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass

    def respond(self, data, content_type='application/json; charset=utf-8', code=200):
        if not isinstance(data, bytes): data = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/': return self.respond(PAGE.read_bytes(), 'text/html; charset=utf-8')
        if self.path != '/api/status': return self.respond({'error':'不存在'}, code=404)
        cameras = {}
        for role, camera in self.server.config['cameras'].items():
            try:
                with OPENER.open(camera['url']+'/status', timeout=1) as response: status = json.load(response)
                cameras[role] = {'serial':status['device']['serial'],
                    'age_ms':status['streams']['color']['age_ms'], 'error':status['error']}
            except Exception as error: cameras[role] = {'serial':camera['serial'],'error':str(error),'age_ms':None}
        session = None
        try:
            session = json.loads((WORKSPACE/'logs/official_record_last.json').read_text())
            info_path = Path(session['root'])/'meta/info.json'
            session['saved_episodes'] = json.loads(info_path.read_text())['total_episodes'] if info_path.exists() else 0
        except (OSError,ValueError,KeyError): pass
        return self.respond({'mode':'preview_only','recording_engine':'official lerobot-record',
            'units':self.server.config['canonical_units'],'can':self.server.bus.snapshot(),
            'cameras':cameras,'last_official_session':session})

    def do_POST(self):
        self.respond({'error':'此页面只预览。录制、重录和保存请使用官方 lerobot-record。'}, code=405)


def main():
    config = json.loads(IO_CONFIG.read_text())
    server = ThreadingHTTPServer(('127.0.0.1',18762),Handler)
    server.config, server.bus = config, PassiveCAN(config)
    print('预览页面：http://127.0.0.1:18762/；录制请运行 record_piper_official.sh',flush=True)
    try: server.serve_forever()
    finally: server.bus.close()


if __name__ == '__main__': main()
