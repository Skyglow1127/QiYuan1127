# Source this file before setup/training on the verified cloud instance.
if [[ -f /etc/network_turbo ]]; then
    source /etc/network_turbo >/dev/null 2>&1
fi
export PIPER_PI05_ROOT=/root/autodl-tmp/piper_pi05
export PI05_BASE_MODEL_PATH="$PIPER_PI05_ROOT/models/pi05_base"
export PI05_TOKENIZER_PATH="$PIPER_PI05_ROOT/models/paligemma_tokenizer"
export HF_HOME="$PIPER_PI05_ROOT/cache/huggingface"
export HF_HUB_CACHE="$HF_HOME/hub"
export HF_HUB_DISABLE_XET=1
export PIP_CACHE_DIR="$PIPER_PI05_ROOT/cache/pip"
export TMPDIR="$PIPER_PI05_ROOT/tmp"
export PATH="$PIPER_PI05_ROOT/.venv/bin:/root/miniconda3/bin:$PATH"
export TOKENIZERS_PARALLELISM=false
export PYTHONUNBUFFERED=1
