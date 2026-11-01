#!/usr/bin/env python3
"""Prepare/upload robot helpers and patched driver source; never start a service.

Default is a local plan/archive. --apply uploads files; --build-driver only
compiles. Python package installation requires its own explicit flag.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tarfile

from fetch_upstream import fetch, git
from teleop_config import load_config, remote_python_command

PROJECT = Path(__file__).resolve().parents[1]
HELPERS = ('g1_upper_body_bridge.py', 'dfx_robot_bridge.py', 'robot_service_runner.py',
           'teleop_config.py', 'g1_camera_server.py', 'probe_g1_state.py', 'build_robot_driver.sh')


def bundle_members(root):
    """An explicit allowlist excludes machine config, reports, keys and environments."""
    members = [(root / 'scripts' / name, name) for name in HELPERS]
    members += [(root / 'upstream.lock.json', 'upstream.lock.json'),
                (root / 'patches/dfx-serial-transactions.patch', 'dfx-serial-transactions.patch')]
    members += [(root / name, name) for name in ('LICENSE', 'NOTICE', 'THIRD_PARTY_NOTICES.md')]
    members += [(path, str(path.relative_to(root))) for path in sorted((root / 'third_party').glob('*')) if path.is_file()]
    for repo, destination in (('DFX_inspire_service', 'DFX_inspire_service'),
                              ('unitree_sdk2_python', 'unitree_sdk2_python')):
        directory = root / 'repos' / repo
        for raw in git(directory, 'ls-files', '-z').split(b'\0'):
            if not raw:
                continue
            relative = Path(raw.decode())
            path = directory / relative
            if relative.is_absolute() or '..' in relative.parts or path.is_symlink() or not path.is_file():
                raise RuntimeError('Unsupported source archive member: ' + str(path))
            members.append((path, str(Path(destination) / relative)))
    for path, _ in members:
        if path.is_symlink() or not path.is_file():
            raise RuntimeError('Missing/unsupported deployment file: ' + str(path))
    return members


def create_bundle(root, archive):
    fetch(root, check=True)
    members = bundle_members(root)
    manifest = {name: hashlib.sha256(path.read_bytes()).hexdigest() for path, name in members}
    archive.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive, 'w:gz') as output:
        for path, name in members:
            output.add(path, arcname=name, recursive=False)
    return manifest


def ssh_command(host, command):
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '--', host, shlex.join(command)]


def main(argv=None):
    config = load_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default=config['G1_SSH_HOST'])
    parser.add_argument('--remote-root', default=config['G1_REMOTE_ROOT'])
    parser.add_argument('--remote-python', default=config['G1_ROBOT_PYTHON'])
    parser.add_argument('--apply', action='store_true', help='upload into configured robot directory')
    parser.add_argument('--build-driver', action='store_true', help='compile inspire_g1 after upload; never run it')
    parser.add_argument('--install-python-sdk', action='store_true', help='pip-install pinned SDK2 source into explicitly chosen robot Python')
    parser.add_argument('--sdk-prefix', default='/usr/local', help='preinstalled robot SDK2 C++ prefix')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args(argv)
    config.update(G1_SSH_HOST=args.host, G1_REMOTE_ROOT=args.remote_root, G1_ROBOT_PYTHON=args.remote_python)
    config = load_config(environ={**config, 'TELEOP_CONFIG': '/dev/null'})
    if args.remote_root == '/' or '..' in Path(args.remote_root).parts:
        parser.error('Use a dedicated absolute robot project directory')
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    archive = PROJECT / '.cache/deploy/robot-source.tar.gz'
    manifest = create_bundle(PROJECT, archive)
    plan = {'host': args.host, 'remote_root': args.remote_root, 'remote_python': args.remote_python,
            'archive': str(archive), 'files_sha256': manifest, 'apply': args.apply,
            'build_driver': args.build_driver, 'install_python_sdk': args.install_python_sdk,
            'services_started': 0, 'note': 'System packages and Unitree SDK2 C++ must already be installed.'}
    report = archive.with_name('deploy-plan.json')
    report.write_text(json.dumps(plan, indent=2) + '\n')
    print(f'Prepared {len(manifest)} files. Plan: {report}')
    if not args.apply:
        print('Local plan only. Add --apply to upload; no SSH connection was opened.')
        return 0
    # Remote shell parses only shlex-quoted argv; archives contain explicit safe relative names.
    subprocess.run(ssh_command(args.host, ['mkdir', '-p', '--', args.remote_root]), check=True)
    with archive.open('rb') as source:
        subprocess.run(ssh_command(args.host, ['tar', '--no-same-owner', '-xzf', '-', '-C', args.remote_root]),
                       stdin=source, check=True)
    if args.install_python_sdk:
        command = remote_python_command(args=['-m', 'pip', 'install', '--disable-pip-version-check',
                                              str(Path(args.remote_root) / 'unitree_sdk2_python')], config=config)
        subprocess.run(ssh_command(args.host, command), check=True)
    check = 'import sys,numpy; assert sys.version_info >= (3,10); import unitree_sdk2py.core.channel; print("Robot Python imports passed; no DDS initialized")'
    subprocess.run(ssh_command(args.host, remote_python_command(args=['-c', check], config=config)), check=True)
    if args.build_driver:
        command = ['bash', str(Path(args.remote_root) / 'build_robot_driver.sh'), '--source',
                   str(Path(args.remote_root) / 'DFX_inspire_service'), '--sdk-prefix', args.sdk_prefix,
                   '--jobs', str(args.jobs)]
        subprocess.run(ssh_command(args.host, command), check=True)
    print('Deployment complete. No services started. Perform read-only checks and explicit commissioning separately.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
