"""Diagnostic wrapper around launch/pursuit.launch.py (no source edits).

Runs the unmodified pursuit launch, but its get_package_share_directory()
points at a temp dir whose config/params.yaml is a copy of the real one with
comm_range / radio_range overridden from env DIAG_COMM_RANGE / DIAG_RADIO_RANGE.
"""
import importlib.util
import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory

_share = get_package_share_directory('boids_swarm')
_tmp = tempfile.mkdtemp(prefix='diag_share_')
os.makedirs(os.path.join(_tmp, 'config'))
_text = open(os.path.join(_share, 'config', 'params.yaml')).read()
for _key, _env in (('comm_range', 'DIAG_COMM_RANGE'),
                   ('radio_range', 'DIAG_RADIO_RANGE')):
    if _env in os.environ:
        _text, _n = re.subn(rf'^(\s*{_key}:\s*)[0-9.]+', rf'\g<1>{os.environ[_env]}',
                            _text, flags=re.M)
        assert _n == 1, (_key, _n)
open(os.path.join(_tmp, 'config', 'params.yaml'), 'w').write(_text)

_spec = importlib.util.spec_from_file_location(
    'orig_pursuit_launch', os.path.join(_share, 'launch', 'pursuit.launch.py'))
_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)
_mod.get_package_share_directory = lambda pkg: (
    _tmp if pkg == 'boids_swarm' else get_package_share_directory(pkg))


def generate_launch_description():
    return _mod.generate_launch_description()
