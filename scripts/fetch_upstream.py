#!/usr/bin/env python3
"""Fetch exact upstream commits and apply reviewed patches, without running them."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
PATCHES = {
    'repos/xr_teleoperate/teleop/robot_control/dex-retargeting': 'patches/dex-conda-pinocchio.patch',
    'repos/DFX_inspire_service': 'patches/dfx-serial-transactions.patch',
}


def git(directory, *args, capture=True):
    return subprocess.run(['git', '-C', str(directory), *args], check=True,
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.PIPE if capture else None).stdout


def relative_path(value):
    path = Path(value)
    if path.is_absolute() or '..' in path.parts or not path.parts:
        raise ValueError('Unsafe manifest path: ' + str(value))
    return path


def expected_patch_files(repo, patch):
    """Derive exact expected bytes from HEAD, not merely a reversible diff."""
    paths = [relative_path(line[6:]) for line in patch.read_text().splitlines() if line.startswith('+++ b/')]
    if not paths or len(paths) != len(set(paths)):
        raise ValueError('Patch must modify distinct existing files: ' + str(patch))
    before = {str(path): git(repo, 'show', 'HEAD:' + path.as_posix()) for path in paths}
    with tempfile.TemporaryDirectory(prefix='teleop-patch-') as temp:
        directory = Path(temp)
        for path, data in before.items():
            destination = directory / path
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
        subprocess.run(['git', 'apply', '--check', str(patch)], cwd=directory, check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        subprocess.run(['git', 'apply', str(patch)], cwd=directory, check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        after = {path: (directory / path).read_bytes() for path in before}
    return before, after


def ensure_patch(repo, patch, check=False):
    if git(repo, 'diff', '--cached', '--name-only').strip():
        raise RuntimeError('Staged changes present; refusing to modify ' + str(repo))
    changed = set(git(repo, 'diff', 'HEAD', '--name-only', '--ignore-submodules=all').decode().splitlines())
    before, after = expected_patch_files(repo, patch)
    actual = {path: (repo / path).read_bytes() for path in before}
    expected_changes = {path for path in before if before[path] != after[path]}
    if actual == after and changed == expected_changes:
        return 'already_applied'
    if actual != before or changed:
        raise RuntimeError('Unexpected tracked edits; use a fresh checkout: ' + str(repo))
    if check:
        raise RuntimeError('Required patch is not applied: ' + str(patch))
    git(repo, 'apply', '--check', str(patch))
    git(repo, 'apply', str(patch))
    return 'applied'


def fetch(root, mirror_root=None, check=False):
    root = root.resolve()
    manifest = json.loads((root / 'upstream.lock.json').read_text())
    records = []
    for entry in manifest['repositories']:
        relative = relative_path(entry['path'])
        commit, url = entry['commit'], entry['url']
        if not re.fullmatch(r'[0-9a-f]{40}', commit) or not url.startswith('https://github.com/'):
            raise ValueError('Expected a full commit and public HTTPS GitHub URL')
        repo = root / relative
        if not (repo / '.git').exists():
            if check:
                raise RuntimeError('Missing checkout: ' + str(repo))
            if repo.exists() and any(repo.iterdir()):
                raise RuntimeError('Refusing nonempty non-Git destination: ' + str(repo))
            repo.mkdir(parents=True, exist_ok=True)
            git(repo, 'init', '--quiet')
            git(repo, 'remote', 'add', 'origin', url)
        try:
            head = git(repo, 'rev-parse', '--verify', 'HEAD').decode().strip()
        except subprocess.CalledProcessError:
            if check or git(repo, 'status', '--porcelain').strip():
                raise RuntimeError('Incomplete checkout has local files: ' + str(repo))
            source = str((mirror_root / relative).resolve()) if mirror_root else 'origin'
            git(repo, 'fetch', '--depth', '1', source, commit, capture=False)
            git(repo, 'checkout', '--quiet', '--detach', commit)
            head = git(repo, 'rev-parse', 'HEAD').decode().strip()
        if head != commit:
            raise RuntimeError(f'{repo}: HEAD {head} differs from pinned {commit}; no reset performed')
        if git(repo, 'remote', 'get-url', 'origin').decode().strip() != url:
            raise RuntimeError('Unexpected origin URL: ' + str(repo))
        record = {'path': str(relative), 'commit': head, 'url': url}
        if str(relative) in PATCHES:
            patch = root / PATCHES[str(relative)]
            record['patch'] = str(patch.relative_to(root))
            record['patch_sha256'] = hashlib.sha256(patch.read_bytes()).hexdigest()
            record['patch_status'] = ensure_patch(repo, patch, check)
        elif git(repo, 'diff', 'HEAD', '--name-only', '--ignore-submodules=all').strip():
            raise RuntimeError('Unexpected tracked edits: ' + str(repo))
        records.append(record)
    return {'hardware_processes_started': 0, 'repositories': records}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=PROJECT)
    parser.add_argument('--mirror-root', type=Path, help='offline source project containing the same repos/ paths')
    parser.add_argument('--check', action='store_true', help='verify only; no fetch or patch application')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    try:
        result = fetch(args.root, args.mirror_root, args.check)
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'Upstream verification failed: {exc}\n')
    report = args.report or args.root / '.cache/upstream-state.json'
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(result, indent=2) + '\n')
    print(f'Verified {len(result["repositories"])} pinned checkouts. Report: {report}')


if __name__ == '__main__':
    main()
