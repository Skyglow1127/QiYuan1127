#!/usr/bin/env bash
set -euo pipefail
piper_act_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$piper_act_root"/{logs,cache,datasets,outputs}
exec > >(tee -a "$piper_act_root/logs/setup.log") 2>&1
export PIP_CACHE_DIR="$piper_act_root/cache/pip"
export PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONHOME
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
if [[ ! -x "$piper_act_root/.venv/bin/python" ]]; then
  /root/miniconda3/bin/python -m venv --system-site-packages "$piper_act_root/.venv"
fi
"$piper_act_root/.venv/bin/python" -m pip install \
  --index-url https://pypi.org/simple \
  -c "$piper_act_root/constraints.txt" \
  'lerobot[training]==0.6.1' 'torchcodec==0.7.0' \
  'av==15.1.0' 'datasets==4.8.5' 'accelerate==1.15.0' 'numpy==2.2.6'
if ! command -v ffmpeg >/dev/null || ! command -v tmux >/dev/null; then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ffmpeg tmux
fi
source "$piper_act_root/env.sh"
python - <<'PY'
import importlib.metadata as m
import torch, torchvision
from torchcodec.decoders import VideoDecoder
assert torch.cuda.is_available(), 'CUDA is required; refusing CPU fallback.'
for name in ['lerobot','torch','torchvision','torchcodec','av','datasets','accelerate','numpy']:
    print(name, m.version(name))
x = torch.randn(128,128,device='cuda',requires_grad=True)
loss = (x @ x).square().mean()
loss.backward()
assert torch.isfinite(x.grad).all()
print('CUDA forward/backward and TorchCodec import passed.')
PY
python -m pip freeze > "$piper_act_root/logs/installed.lock.txt"
printf 'ACT cloud environment ready. Source %s/env.sh\n' "$piper_act_root"
