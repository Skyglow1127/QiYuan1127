"""Download pinned public checkpoints. No robot or camera access."""
import hashlib
import json
from pathlib import Path

from huggingface_hub import HfApi, snapshot_download

ROOT = Path('/home/aup/模型库/策略模型/SmolVLA')
BASE_REVISION = '5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6'
SPECS = [
    ('lerobot/smolvla_base', BASE_REVISION, 'base', None),
    ('HuggingFaceTB/SmolVLM2-500M-Video-Instruct', None, 'vlm_processor',
     ['*.safetensors', '*.bin', '*.pt', '*.pth', '*.onnx']),
]

def main():
    manifest = []
    api = HfApi()
    for repo, revision, folder, ignore in SPECS:
        info = api.model_info(repo, revision=revision)
        dest = ROOT / folder
        print(f'Downloading {repo}@{info.sha} -> {dest}', flush=True)
        snapshot_download(repo_id=repo, revision=info.sha, local_dir=dest,
                          ignore_patterns=ignore, max_workers=2)
        files = []
        for p in sorted(dest.rglob('*')):
            if not p.is_file() or '.cache' in p.parts:
                continue
            h = hashlib.sha256()
            with p.open('rb') as f:
                for data in iter(lambda: f.read(8 * 1024 * 1024), b''):
                    h.update(data)
            files.append({'path': str(p.relative_to(dest)), 'bytes': p.stat().st_size,
                          'sha256': h.hexdigest()})
        manifest.append({'repo': repo, 'revision': info.sha, 'directory': str(dest),
                         'ignored': ignore, 'files': files})
        (ROOT / 'download_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        print(f'Verified {len(files)} files for {repo}', flush=True)

if __name__ == '__main__':
    main()
