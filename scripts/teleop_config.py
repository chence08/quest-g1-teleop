"""Portable deployment settings; config files are data, never shell scripts.

Load project config/local.env (or TELEOP_CONFIG), then override with environment values.
Every remote Python command explicitly forwards these settings over SSH.
"""
import ipaddress
import os
from pathlib import Path
import re
import shlex

PROJECT_ROOT = Path(__file__).resolve().parent
if PROJECT_ROOT.name == 'scripts':
    PROJECT_ROOT = PROJECT_ROOT.parent

DEFAULTS = {
    'G1_REMOTE_ROOT': '/home/unitree/quest-g1-teleop',
    'G1_ROBOT_PYTHON': 'python3',
    'G1_CAMERA_PYTHON': 'python3',
    'G1_SSH_HOST': 'g1',
    'HOST_LAN_IP': '127.0.0.1',
    'G1_DDS_NAMESPACE': 'inspire_teleop',
    'G1_ARM_INTERFACE': 'enP8p1s0',
    'G1_HAND_INTERFACE': 'lo',
}


def load_config(environ=None, config_path=None):
    environment = os.environ if environ is None else environ
    result = dict(DEFAULTS)
    explicit_path = config_path if config_path is not None else environment.get('TELEOP_CONFIG')
    path = Path(explicit_path).expanduser() if explicit_path else PROJECT_ROOT / 'config/local.env'
    if path.is_file() or path == Path('/dev/null'):
        for number, raw in enumerate(path.read_text().splitlines(), 1):
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            if '=' not in line:
                raise ValueError(f'{path}:{number}: expected KEY=VALUE')
            key, raw_value = line.split('=', 1)
            key = key.strip()
            if key not in DEFAULTS:
                raise ValueError(f'{path}:{number}: unknown setting {key!r}')
            value = shlex.split(raw_value, comments=True, posix=True)
            if len(value) != 1:
                raise ValueError(f'{path}:{number}: use one value, quoting paths containing spaces')
            result[key] = value[0]
    elif explicit_path:
        raise ValueError(f'Configuration file does not exist: {path}')
    result.update({key: environment[key] for key in DEFAULTS if key in environment})
    for key, value in result.items():
        if not value or any(character in value for character in ('\x00', '\n', '\r')):
            raise ValueError(f'{key} must be a nonempty single-line value')
    if not Path(result['G1_REMOTE_ROOT']).is_absolute():
        raise ValueError('G1_REMOTE_ROOT must be an absolute robot path')
    if not re.fullmatch(r'[A-Za-z0-9_]+', result['G1_DDS_NAMESPACE']):
        raise ValueError('G1_DDS_NAMESPACE must contain only letters, digits and underscores')
    if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', result['G1_SSH_HOST']) or result['G1_SSH_HOST'].startswith('-'):
        raise ValueError('G1_SSH_HOST must be an SSH alias/host, not options or shell text')
    return result


def remote_python_command(script_name=None, args=(), config=None):
    """Return shell-quotable argv with explicit settings for the robot process."""
    settings = load_config() if config is None else config
    command = ['env', 'PYTHONNOUSERSITE=1', 'TELEOP_CONFIG=/dev/null']
    command += [f'{key}={settings[key]}' for key in DEFAULTS]
    command += [settings['G1_ROBOT_PYTHON'], '-u']
    if script_name is not None:
        if Path(script_name).name != script_name:
            raise ValueError('Remote script must be a basename')
        command.append(str(Path(settings['G1_REMOTE_ROOT']) / script_name))
    return command + [str(value) for value in args]


def validate_host_ip(parser, value, execute=False):
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError:
        parser.error('--host-ip / HOST_LAN_IP must be a workstation IPv4 address')
    if execute and (address.is_loopback or address.is_unspecified or address.is_multicast):
        parser.error('Live execution requires --host-ip or HOST_LAN_IP set to the workstation LAN address reachable from Quest')
