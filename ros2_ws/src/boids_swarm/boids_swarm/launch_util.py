"""Launch-side helpers shared by pursuit.launch.py, nav2_target.launch.py
and the tests (pure Python, no ROS).

The single source of truth for obstacles is `obstacles_for_env`: the sim,
the controllers and the Nav2 bridge all receive the SAME flat list
(`[x, y, r, ...]`, sentinel `[0.0]` when empty), so the map the Nav2
costmap plans on can never drift from the world the sim collides with.
"""

import signal

from .world_gen import WorldGenerator

EMPTY = [0.0]


def parse_obstacles(text: str):
    """'x,y,r;x,y,r' -> flat [x, y, r, ...]. Empty -> sentinel [0.0]."""
    flat = []
    for chunk in text.split(';'):
        parts = [p for p in chunk.replace(' ', '').split(',') if p]
        if len(parts) == 3:
            flat += [float(v) for v in parts]
    return flat if flat else list(EMPTY)


def flat_to_obstacles(flat):
    flat = [float(v) for v in flat]
    if len(flat) < 3:
        return []
    return [tuple(flat[i:i + 3]) for i in range(0, len(flat) - 2, 3)]


def format_obstacles(flat) -> str:
    """Inverse of parse_obstacles with exact float round-trip (repr)."""
    return ';'.join(f'{x!r},{y!r},{r!r}' for (x, y, r)
                    in flat_to_obstacles(flat))


def obstacles_for_env(env: str, seed: int, world_size: float,
                      custom_text: str = ''):
    """Flat obstacle list for a launch: procedural env or the custom string."""
    if env == 'custom':
        return parse_obstacles(custom_text)
    return WorldGenerator(int(seed), float(world_size)).generate(
        env).obstacles_flat()


def load_ros_params(params_file):
    """The flat `ros__parameters` dict of a params.yaml ('/**' wildcard)."""
    import yaml
    try:
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        return dict(doc['/**']['ros__parameters'])
    except (OSError, KeyError, TypeError):
        return {}


def target_limits(params, overrides=None):
    """(v_max, w_max, body_radius) the SIM will enforce on the target, so
    Nav2's controller limits derive from the same parameters instead of
    being hard-coded (v_max = agent_max_speed * target_speed_multiplier)."""
    p = dict(params)
    p.update(overrides or {})
    v = float(p.get('agent_max_speed', 2.0)) * \
        float(p.get('target_speed_multiplier', 2.0))
    return (v, float(p.get('target_omega_max', 2.5)),
            float(p.get('target_body_radius', 0.7)))


def with_robot_radius(doc, radius):
    """Copy of a Nav2 parameter doc whose global AND local costmap use
    `radius` as robot_radius (the target's body radius from params.yaml),
    so the planner footprint can never drift from the body the sim collides
    with."""
    import copy
    out = copy.deepcopy(doc)
    for sect in ('global_costmap', 'local_costmap'):
        out[sect][sect]['ros__parameters']['robot_radius'] = float(radius)
    return out


def write_derived_nav2_config(cfg_path, radius):
    """Write `cfg_path` with robot_radius := radius to a temp yaml, return it."""
    import tempfile
    import yaml
    with open(cfg_path) as f:
        doc = yaml.safe_load(f)
    fd, path = tempfile.mkstemp(prefix='nav2_target_', suffix='.yaml')
    with open(fd, 'w') as f:
        yaml.safe_dump(with_robot_radius(doc, radius), f)
    return path


def ensure_sigint_deliverable(getsignal=signal.getsignal,
                              install=signal.signal):
    """Make Ctrl-C / `kill -INT` reach `ros2 launch` when it was started
    with SIGINT already ignored. Returns True if it changed anything.

    A process started from a non-interactive shell as a background job
    (`cmd &`), under nohup-style wrappers or from many CI/automation
    runners inherits SIGINT = SIG_IGN. Python then does not install its
    own SIGINT handler, and launch's AsyncSafeSignalManager (which listens
    through signal.set_wakeup_fd, and so needs *a* Python-level handler to
    exist) never hears the signal: the launch, and so the sim window and
    the whole stack, keep running until a SIGTERM. The handler installed
    here does nothing itself; launch's wakeup fd does the shutdown."""
    if getsignal(signal.SIGINT) != signal.SIG_IGN:
        return False
    install(signal.SIGINT, lambda *_: None)
    return True
