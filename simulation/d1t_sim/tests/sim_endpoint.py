"""Use the same local simulation endpoint as the launcher."""
import json, os
from pathlib import Path
_root = Path(__file__).resolve().parents[1]
_port = int(os.environ.get('D1T_PORT') or json.loads((_root / 'config/runtime.json').read_text())['simulation_port'])
BASE_URL = f'http://127.0.0.1:{_port}'
