"""Owned loopback-only llama.cpp process for memory-efficient local Qwen vision.

No image or prompt is sent to an external service. close() terminates only the
child process created by this instance. JSON grammar is optional for benchmarks.
"""
import base64
import copy
import io
import json
from pathlib import Path
import socket
import secrets
import subprocess
import tempfile
import time
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
OBJECT_SCHEMA = {
    'type': 'array', 'maxItems': 8,
    'items': {'type': 'object', 'properties': {
        'bbox_2d': {'type': 'array', 'minItems': 4, 'maxItems': 4,
                    'items': {'type': 'integer', 'minimum': 0, 'maximum': 1000}},
        'label': {'type': 'string', 'minLength': 1, 'maxLength': 120}},
        'required': ['bbox_2d', 'label'], 'additionalProperties': False}}


class LlamaRuntime:
    def __init__(self, model_path, threads=4, max_edge=640, mmproj=None, server_binary=None,
                 log_path=None, strict_json=True, device='cpu'):
        model_path = Path(model_path).resolve()
        mmproj = Path(mmproj).resolve() if mmproj else model_path.parent/'mmproj-bf16.gguf'
        if device not in ('cpu','vulkan'): raise ValueError('Device must be cpu or vulkan')
        backend_build = 'build-vulkan' if device=='vulkan' else 'build'
        server_binary = Path(server_binary or ROOT/'experiments/llama.cpp'/backend_build/'bin/llama-server').resolve()
        if not 1 <= threads <= 24 or not 256 <= max_edge <= 1280:
            raise ValueError('Invalid runtime bounds')
        for path in (model_path, mmproj, server_binary):
            if not path.is_file():
                raise FileNotFoundError(path)
        self.path, self.mmproj = model_path, mmproj
        self.threads, self.max_edge, self.strict_json = threads, max_edge, strict_json
        self.object_limit = 6
        self.device = device
        self.startup_fallback_reason = None
        self.api_key = secrets.token_urlsafe(32)
        self.process = None
        self.log = open(log_path, 'w') if log_path else tempfile.TemporaryFile(mode='w+')
        # Never follow proxy environment variables for a private local request.
        self.http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        self.base_url = f'http://127.0.0.1:{port}'
        command = [str(server_binary), '-m', str(model_path), '--mmproj', str(mmproj),
                   '--host', '127.0.0.1', '--port', str(port), '--no-webui',
                   '--api-key', self.api_key, '-t', str(threads), '-tb', str(threads),
                   '-np', '1', '-c', '2048', '--image-max-tokens', '320',
                   '--cache-ram', '0', '--no-cache-idle-slots',
                   '--reasoning', 'off', '--reasoning-budget', '0',
                   '--chat-template-kwargs', '{"enable_thinking":false}']
        command += ['-ngl','99','--device','Vulkan0'] if device=='vulkan' else ['-ngl','0','--no-mmproj-offload']
        started = time.perf_counter()
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self.log, stderr=self.log)
            deadline = time.monotonic()+60
            while time.monotonic() < deadline:
                if self.process.poll() is not None:
                    raise RuntimeError('Local Qwen server exited during initialization; see runtime log')
                try:
                    health = urllib.request.Request(self.base_url+'/health',headers={'Authorization':'Bearer '+self.api_key})
                    with self.http.open(health, timeout=1) as response:
                        if response.status == 200:
                            break
                except (urllib.error.URLError, TimeoutError):
                    time.sleep(.1)
            else:
                raise TimeoutError('Local Qwen server initialization timed out')
        except BaseException:
            self.close()
            raise
        self.load_ms = 1000*(time.perf_counter()-started)

    def infer(self, image, prompt, max_tokens=160):
        if not isinstance(prompt, str) or not prompt.strip() or len(prompt)>8000 or not 16<=max_tokens<=512:
            raise ValueError('Invalid bounded inference request')
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('Local Qwen server is not running')
        started = time.perf_counter()
        view = image.convert('RGB').copy()
        view.thumbnail((self.max_edge, self.max_edge))
        encoded = io.BytesIO(); view.save(encoded, format='PNG')
        data_url = 'data:image/png;base64,'+base64.b64encode(encoded.getvalue()).decode('ascii')
        payload = {'messages': [{'role': 'user', 'content': [
            {'type': 'image_url', 'image_url': {'url': data_url}}, {'type': 'text', 'text': prompt}]}],
            'temperature': 0, 'max_tokens': max_tokens, 'cache_prompt': False, 'stream': False,
            'chat_template_kwargs': {'enable_thinking': False}, 'reasoning_format': 'deepseek'}
        if self.strict_json:
            schema = copy.deepcopy(OBJECT_SCHEMA)
            schema['maxItems'] = self.object_limit
            payload['response_format'] = {'type': 'json_schema', 'json_schema': {'name': 'objects', 'schema': schema}}
        request = urllib.request.Request(self.base_url+'/v1/chat/completions',
                                         data=json.dumps(payload).encode(), headers={'Content-Type': 'application/json','Authorization':'Bearer '+self.api_key})
        try:
            with self.http.open(request, timeout=90) as response:
                result = json.load(response)
        except urllib.error.HTTPError as error:
            detail = error.read(4096).decode('utf-8', errors='replace')
            raise RuntimeError(f'Local Qwen HTTP {error.code}: {detail}') from error
        choice = result['choices'][0]
        text = choice['message']['content']
        if not isinstance(text, str):
            raise ValueError('Server returned no textual result')
        usage = result.get('usage', {})
        memory = {}
        try:
            for line in Path(f'/proc/{self.process.pid}/status').read_text().splitlines():
                if line.startswith('VmHWM:'): memory['server_peak_rss_kib'] = int(line.split()[1])
                if line.startswith('VmRSS:'): memory['server_current_rss_kib'] = int(line.split()[1])
        except (OSError, ValueError): pass
        return text.strip(), {'model_load_ms': self.load_ms, 'grounding_ms': 1000*(time.perf_counter()-started),
            'generated_tokens': usage.get('completion_tokens'), 'input_tokens': usage.get('prompt_tokens'),
            'input_image_size': list(view.size), 'runtime': 'llama.cpp '+self.device, 'model': str(self.path),
            'startup_fallback_reason': self.startup_fallback_reason,
            'mmproj': str(self.mmproj), 'threads': self.threads, 'max_edge': self.max_edge,
            'strict_json': self.strict_json, 'prompt_cache': False,
            'server_prompt_cache_ram_mib': 0,
            'generation_hit_token_limit': choice.get('finish_reason') == 'length',
            'server_timings': result.get('timings', {}), **memory}

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait(timeout=5)
            self.process = None
        if getattr(self, 'log', None) is not None:
            self.log.close(); self.log = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def create_llama_runtime(model_path, threads=4, device='auto', **kwargs):
    """Prefer a prepared local Vulkan backend; preserve an explicit CPU fallback."""
    import os
    if device not in ('auto','cpu','vulkan'): raise ValueError('Invalid device selection')
    ready = ((ROOT/'experiments/llama.cpp/build-vulkan/bin/llama-server').is_file() and
             os.access('/dev/dri/renderD128',os.R_OK|os.W_OK))
    selected = 'vulkan' if device=='vulkan' or (device=='auto' and ready) else 'cpu'
    try:
        return LlamaRuntime(model_path,threads,device=selected,**kwargs)
    except Exception as error:
        if device!='auto' or selected!='vulkan': raise
        log_path = kwargs.get('log_path')
        if log_path and Path(log_path).exists():
            Path(log_path).replace(Path(log_path).with_suffix('.vulkan-failed.log'))
        runtime = LlamaRuntime(model_path,threads,device='cpu',**kwargs)
        runtime.startup_fallback_reason = str(error)
        return runtime
