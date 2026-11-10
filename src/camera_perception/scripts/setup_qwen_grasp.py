"""Rebuild the tested local Qwen/SAM assets from pinned sources and manifests."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
LLAMA_COMMIT = '2a667032dc788c3a071d538e0b572e45a3f12731'


def sha256(path):
    with Path(path).open('rb') as source: return hashlib.file_digest(source,'sha256').hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--check-only', action='store_true', help='Verify existing assets without downloading or compiling')
    p.add_argument('--profile', choices=['balanced','review','all'], default='balanced')
    a = p.parse_args()
    manifest_path = ROOT/'models/qwen_grasp_runtime.json'
    if not manifest_path.is_file(): p.error('Runtime manifest is missing')
    runtime = json.loads(manifest_path.read_text())
    variants = [v for v in runtime['variants'] if a.profile=='all' or
                v['repo'].endswith('2B' if a.profile=='balanced' else '4B')]
    source = ROOT/'experiments/llama.cpp'
    if not a.check_only:
        if not source.exists():
            source.parent.mkdir(exist_ok=True)
            subprocess.run(['git','clone','--no-checkout','--filter=blob:none','https://github.com/ggml-org/llama.cpp.git',str(source)],check=True)
            subprocess.run(['git','checkout','--detach',LLAMA_COMMIT],cwd=source,check=True)
        revision = subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
        if revision!=LLAMA_COMMIT or subprocess.check_output(['git','status','--porcelain'],cwd=source,text=True).strip():
            raise RuntimeError('Existing llama.cpp source differs from tested revision; preserve it before preparing a clean checkout')
        subprocess.run(['cmake','-S',str(source),'-B',str(source/'build'),'-DCMAKE_BUILD_TYPE=Release',
                        '-DGGML_NATIVE=ON','-DGGML_OPENMP=ON','-DLLAMA_BUILD_TESTS=OFF'],check=True)
        subprocess.run(['cmake','--build',str(source/'build'),'--target','llama-server','llama-quantize','-j','4'],check=True)
        from huggingface_hub import snapshot_download
        for variant in variants:
            original = ROOT/variant['hf_directory']
            weights_manifest = json.loads((ROOT/variant['hf_manifest']).read_text())
            hashes = weights_manifest['files_sha256']
            if any(not (original/name).is_file() for name in hashes):
                snapshot_download(variant['repo'],revision=weights_manifest['revision'],local_dir=original,
                                  allow_patterns=list(hashes),max_workers=2)
            for name,expected in hashes.items():
                if sha256(original/name)!=expected: raise RuntimeError('Original weight mismatch: '+name)
            directory = ROOT/variant['gguf_directory']; directory.mkdir(exist_ok=True)
            text = directory/'model-bf16.gguf'; vision = directory/'mmproj-bf16.gguf'
            quantized = directory/variant['quantized_filename']
            if not quantized.exists() and not text.exists():
                subprocess.run([sys.executable,str(source/'convert_hf_to_gguf.py'),str(original),
                                '--outfile',str(text),'--outtype','bf16'],check=True)
            if not vision.exists():
                subprocess.run([sys.executable,str(source/'convert_hf_to_gguf.py'),str(original),
                                '--mmproj','--outfile',str(vision),'--outtype','bf16'],check=True)
            if not quantized.exists():
                subprocess.run([str(source/'build/bin/llama-quantize'),str(text),str(quantized),variant['quantization'],'4'],check=True)
        sam = ROOT/'models/sam2.1_t.pt'
        sam_manifest = json.loads((ROOT/'models/sam2.1_t.json').read_text())
        if not sam.exists():
            temporary = sam.with_suffix('.download')
            urllib.request.urlretrieve(sam_manifest['source'],temporary)
            if sha256(temporary)!=sam_manifest['sha256']: raise RuntimeError('SAM download checksum mismatch')
            temporary.replace(sam)
    for name,expected in runtime['files_sha256'].items():
        if name!='models/sam2.1_t.pt' and not any(name.startswith(v['gguf_directory']+'/') for v in variants): continue
        if sha256(ROOT/name)!=expected: raise RuntimeError('Runtime asset checksum mismatch: '+name)
    if not (source/'build/bin/llama-server').is_file(): raise FileNotFoundError('llama-server is not built')
    print('All pinned Qwen/SAM runtime assets verified.')


if __name__ == '__main__': main()
