#!/usr/bin/env python3
"""Download immutable published checkpoints, verify hashes, write local asset paths."""
import argparse
import json
from pathlib import Path
from publish_models import ROOT, MANIFEST, DEFAULTS, digest, write_json

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--models', nargs='+', default=DEFAULTS)
    p.add_argument('--output', type=Path, default=ROOT / '.local/downloaded-models')
    p.add_argument('--assets-output', type=Path, default=ROOT / '.local/downloaded-assets.json')
    p.add_argument('--list', action='store_true', help='Show status without downloading')
    a = p.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    items = {m['id']: m for m in manifest['models']}
    if a.list:
        for item in items.values():
            print(item['id'], item['publication_status'], item.get('repo_url', ''))
        return
    for mid in a.models:
        if mid not in items or items[mid]['publication_status'] != 'published':
            p.error(f'Model is unknown or unpublished: {mid}')
    from huggingface_hub import snapshot_download
    assets = json.loads(a.assets_output.read_text()) if a.assets_output.exists() else {}
    for mid in a.models:
        item = items[mid]
        dest = a.output.resolve() / mid
        # Do not write through links into original engineering directories.
        if dest.is_symlink():
            raise ValueError(f'Refusing symlink destination: {dest}')
        for name in item['files']:
            candidate = dest / name
            if not candidate.resolve().is_relative_to(dest.resolve()) or candidate.is_symlink():
                raise ValueError(f'Unsafe destination: {candidate}')
        snapshot_download(item['repo_id'], revision=item['revision'],
                          allow_patterns=list(item['files']), local_dir=dest)
        for name, expected in item['files'].items():
            f = dest / name
            if f.stat().st_size != expected['bytes'] or digest(f) != expected['sha256']:
                raise ValueError(f'Download integrity check failed: {mid}/{name}')
        assets[mid] = str(dest)
        if mid == 'pi05_best004000':
            assets['tokenizer'] = str(dest / 'tokenizer')
        write_json(a.assets_output, assets)
        print(f'Verified: {mid} -> {dest}', flush=True)
    print(f'Asset mapping: {a.assets_output}')

if __name__ == '__main__':
    main()
