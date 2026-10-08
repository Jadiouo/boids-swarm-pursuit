"""Record what a launch file *would* start, without starting anything.

Used by test_launch_compat to prove that pursuit.launch.py on the
headless / ui:=false path starts exactly the nodes (same parameters) it did
before the control-panel work, and that the UI path's swarm_stack.launch.py
starts the same controllers.

Launch actions (`Node`, `TimerAction`, ...) are replaced by recorders in
the launch module under test, `launch_setup` is called with a real
`LaunchContext`, and the recorded calls are normalised to plain JSON-able
data. Requires `launch` (ROS), so the importer skips when it is missing.
"""

import importlib.util
import json
import os

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys  # noqa: E402
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)


def _norm(v):
    if isinstance(v, dict):
        return {str(k): _norm(x) for k, x in sorted(v.items(),
                                                    key=lambda kv: str(kv[0]))}
    if isinstance(v, (list, tuple)):
        return [_norm(x) for x in v]
    if isinstance(v, float):
        return round(v, 9)
    if isinstance(v, (int, str, bool)) or v is None:
        if isinstance(v, str) and os.path.isabs(v):
            return os.path.basename(v)          # params.yaml path etc.
        return v
    return type(v).__name__                    # Shutdown() and friends


class Rec:
    def __init__(self, kind, **kw):
        self.kind, self.kw = kind, kw

    def data(self):
        out = {'kind': self.kind}
        for k, v in self.kw.items():
            if k == 'actions':
                out[k] = [a.data() for a in v]
            elif k == 'launch_arguments':
                out[k] = _norm(dict(v))
            else:
                out[k] = _norm(v)
        return out


def _node(**kw):
    return Rec('node', **kw)


def _timer(period=None, actions=None, **kw):
    return Rec('timer', period=period, actions=actions or [], **kw)


def _include(source=None, launch_arguments=(), **kw):
    return Rec('include', source=getattr(source, 'name', 'src'),
               launch_arguments=launch_arguments)


class _Src:
    def __init__(self, path):
        self.name = os.path.basename(path)


def load_launch(name):
    path = os.path.join(PKG_DIR, 'launch', name)
    spec = importlib.util.spec_from_file_location(
        'rec_' + name.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _patch(mod):
    for attr, repl in (('Node', _node), ('TimerAction', _timer),
                       ('IncludeLaunchDescription', _include),
                       ('PythonLaunchDescriptionSource', _Src),
                       ('get_package_share_directory',
                        lambda _pkg: PKG_DIR)):
        if hasattr(mod, attr):
            setattr(mod, attr, repl)


def record(launch_name, overrides):
    from launch import LaunchContext
    mod = load_launch(launch_name)
    _patch(mod)
    extra = []
    try:
        import boids_swarm.stack_launch as sl
        extra.append(sl)
    except ImportError:
        pass
    saved = []
    for m in extra:
        saved.append({a: getattr(m, a) for a in
                      ('Node', 'TimerAction', 'IncludeLaunchDescription',
                       'PythonLaunchDescriptionSource',
                       'get_package_share_directory') if hasattr(m, a)})
        _patch(m)
    try:
        ld = mod.generate_launch_description()
        ctx = LaunchContext()
        for ent in ld.entities:
            if hasattr(ent, 'name') and hasattr(ent, 'default_value'):
                dv = ent.default_value
                ctx.launch_configurations[ent.name] = \
                    dv[0].perform(ctx) if dv else ''
        for k, v in overrides.items():
            ctx.launch_configurations[k] = v
        acts = mod.launch_setup(ctx)
        return [a.data() for a in acts]
    finally:
        for m, s in zip(extra, saved):
            for a, v in s.items():
                setattr(m, a, v)


def dumps(obj):
    return json.dumps(obj, indent=1, sort_keys=True)
