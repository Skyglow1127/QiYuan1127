#!/usr/bin/env python3
"""Verify the supplemental source snapshot without importing it or using hardware."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    manifest = json.loads((ROOT / 'docs/workspace_audit_20261111.json').read_text())
    errors = []
    for item in manifest['files']:
        path = ROOT / item['path']
        if path.is_symlink() or not path.resolve().is_relative_to(ROOT):
            errors.append(f'Unsafe path: {item["path"]}')
        elif not path.is_file():
            errors.append(f'Missing: {item["path"]}')
        else:
            data = path.read_bytes()
            if len(data) != item['bytes'] or hashlib.sha256(data).hexdigest() != item['sha256']:
                errors.append(f'Changed: {item["path"]}')
    print(json.dumps({'files_checked': len(manifest['files']), 'errors': errors},
                     ensure_ascii=False, indent=2))
    return bool(errors)


if __name__ == '__main__':
    sys.exit(main())
