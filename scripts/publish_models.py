#!/usr/bin/env python3
"""Stage selected checkpoints; --upload publishes and records verified HF revisions."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'models/manifest.json'
DEFAULTS = ['act_020000', 'pi05_best004000']
NAMES = {'act_020000': 'QiYuan1127-PiPER-ACT-020000',
         'pi05_best004000': 'QiYuan1127-PiPER-PI05-best004000',
         'pi05_final020000': 'QiYuan1127-PiPER-PI05-final020000'}

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)

def stage(item, locations):
    mid = item['id']
    dest = ROOT / '.local/hf-staging' / mid
    dest.mkdir(parents=True, exist_ok=True)
    source = Path(locations[mid]) / item['file']
    if source.stat().st_size != item['bytes']:
        raise ValueError(f'{mid}: weight size mismatch')
    target = dest / item['file']
    if target.is_symlink():
        if target.resolve() != source.resolve():
            raise ValueError(f'Unexpected staging link: {target}')
    elif target.exists():
        raise ValueError(f'Refusing to replace {target}')
    else:
        target.symlink_to(source.resolve())
    for f in (ROOT / item['config_directory']).iterdir():
        if f.is_file():
            shutil.copy2(f, dest / f.name)
    if mid.startswith('pi05'):
        for name in ['GEMMA_TERMS.txt', 'GEMMA_PROHIBITED_USE_POLICY.txt', 'NOTICE.txt']:
            shutil.copy2(ROOT / 'models/publication' / name, dest / name)
        tok = dest / 'tokenizer'
        tok.mkdir(exist_ok=True)
        expected = json.loads((ROOT / 'src/pi05/pi05_expert_inference_v6/local_config.json').read_text())['expected_tokenizer_sha256']
        for name, sha in expected.items():
            original = Path(locations['tokenizer']) / name
            if digest(original) != sha:
                raise ValueError(f'Tokenizer hash mismatch: {name}')
            shutil.copy2(original, tok / name)
    files = {}
    for f in sorted(dest.rglob('*')):
        if (not f.is_file() or '.cache' in f.relative_to(dest).parts
                or f.name.lower().startswith('readme')
                or f.suffix.lower() in {'.md', '.markdown', '.mdx'}):
            continue
        name = f.relative_to(dest).as_posix()
        files[name] = {'bytes': f.stat().st_size,
                       'sha256': item['sha256'] if name == item['file'] else digest(f)}
    return dest, files

def publish(api, item, dest, files, namespace):
    from huggingface_hub import hf_hub_download, hf_hub_url
    repo = namespace + '/' + NAMES[item['id']]
    if api.repo_exists(repo):
        old = api.model_info(repo, files_metadata=True)
        if old.private:
            raise ValueError(f'Existing repository is private; refusing to change visibility: {repo}')
        for f in old.siblings:
            if f.rfilename == item['file'] and (not f.lfs or f.lfs.sha256 != item['sha256']):
                raise ValueError(f'Existing different weight in {repo}; refusing to overwrite')
    else:
        api.create_repo(repo, repo_type='model', private=False)
    print(f'Uploading {repo}', flush=True)
    commit = api.upload_folder(repo_id=repo, folder_path=dest,
                               allow_patterns=list(files),
                               commit_message='Publish verified PiPER checkpoint and processors')
    # Anonymous metadata and small-file downloads prove public accessibility.
    from huggingface_hub import HfApi
    info = HfApi(token=False).model_info(repo, revision=commit.oid, files_metadata=True)
    remote = {f.rfilename: f for f in info.siblings}
    for name, expected in files.items():
        f = remote.get(name)
        if f is None or f.size != expected['bytes']:
            raise ValueError(f'Remote size mismatch: {repo}/{name}')
        if f.lfs:
            sha = f.lfs.sha256
        else:
            sha = digest(Path(hf_hub_download(repo, name, revision=commit.oid, token=False)))
        if sha != expected['sha256']:
            raise ValueError(f'Remote hash mismatch: {repo}/{name}')
    item.update(publication_status='published', repo_id=repo, revision=commit.oid,
                repo_url='https://huggingface.co/' + repo,
                download_url=hf_hub_url(repo, item['file'], revision=commit.oid), files=files)
    print(f'Verified {repo}@{commit.oid}', flush=True)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets', type=Path, default=ROOT / '.local/artifact_locations.json')
    p.add_argument('--models', nargs='+', choices=list(NAMES), default=DEFAULTS)
    p.add_argument('--upload', action='store_true', help='Create public HF repositories and upload')
    p.add_argument('--namespace', help='Defaults to the authenticated HF username')
    a = p.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    locations = json.loads(a.assets.read_text())
    selected = [x for x in manifest['models'] if x['id'] in a.models]
    prepared = [(item, *stage(item, locations)) for item in selected]
    write_json(ROOT / '.local/hf-publication-plan.json', [
        {'id': item['id'], 'repository_name': NAMES[item['id']], 'files': files,
         'bytes': sum(x['bytes'] for x in files.values())} for item, dest, files in prepared])
    print('Prepared: ' + ', '.join(x['id'] for x in selected), flush=True)
    if not a.upload:
        print('Local preparation only; no network changes. Use --upload after hf auth login.')
        return
    from huggingface_hub import HfApi
    api = HfApi()
    namespace = a.namespace or api.whoami()['name']
    for item, dest, files in prepared:
        publish(api, item, dest, files, namespace)
        if item['id'] == 'pi05_best004000':
            manifest['tokenizer'].update(publication_status='published', repo_id=item['repo_id'],
                revision=item['revision'], subdirectory='tokenizer',
                download_url=item['repo_url'] + '/tree/' + item['revision'] + '/tokenizer')
        write_json(MANIFEST, manifest)

if __name__ == '__main__':
    main()
