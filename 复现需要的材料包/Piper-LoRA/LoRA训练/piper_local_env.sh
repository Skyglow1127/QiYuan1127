#!/usr/bin/env bash
# Source this file to select the local PI05 runtime and preloaded assets.
export PIPER_PI05_ROOT="${PIPER_PI05_ROOT:-/home/aup/Lerobot}"
export PI05_BASE_MODEL_PATH="${PI05_BASE_MODEL_PATH:-$PIPER_PI05_ROOT/models/PI05/pi05_base}"
export PI05_TOKENIZER_PATH="${PI05_TOKENIZER_PATH:-$PIPER_PI05_ROOT/models/PI05/paligemma_tokenizer}"
export PIPER_PI05_PYTHON="${PIPER_PI05_PYTHON:-$PIPER_PI05_ROOT/envs/pi05-rocm-7.14.1/bin/python}"
export HF_HOME="${HF_HOME:-$PIPER_PI05_ROOT/cache/pi05/huggingface}"
export HF_HUB_CACHE="${HF_HUB_CACHE:-$HF_HOME/hub}"
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export HF_HUB_DISABLE_XET=1
export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$PIPER_PI05_ROOT/cache/pi05/pip}"
export TMPDIR="${TMPDIR:-$PIPER_PI05_ROOT/work/pi05/tmp}"
export TOKENIZERS_PARALLELISM=false
export PYTHONUNBUFFERED=1
