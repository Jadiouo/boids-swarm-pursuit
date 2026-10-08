"""Modes, stack arguments and validation for the in-window control panel.

Pure Python (no ROS, no pygame) so the rules are unit-testable and CI-able.

Vocabulary
    stack     the processes the panel can restart without closing the window:
              N x boid_controller + target_controller (+ nav2_bridge and the
              Nav2 servers when evader=nav2). Launched by
              `swarm_stack.launch.py`; the pygame sim (window owner) is NOT
              part of it.
    mode      a named preset of the restart-needing settings
              (perception / sharing_mode / evader).
    cfg       the restart-needing settings the sim currently runs with
              (RESTART_KEYS). Everything else is "live": changed with a
              parameter call, no restart.

`stack_args` is the single place that turns (mode, overrides) into the
launch arguments of `swarm_stack.launch.py`; the same keys are what
pursuit.launch.py hands to the sim in UI mode, so the first stack and every
restarted stack are built by the same code.
"""

import json

# ---------------------------------------------------------------------------
# modes
BASELINE = 'baseline'
SENSOR_ROS = 'sensor_ros'
NAV2 = 'nav2'
MODES = (BASELINE, SENSOR_ROS, NAV2)

MODE_LABELS = {BASELINE: 'Baseline', SENSOR_ROS: 'Sensor + ROS relay',
               NAV2: 'Nav2 target'}
# Short labels for the narrow button row.
MODE_SHORT = {BASELINE: 'Baseline', SENSOR_ROS: 'Sensor+ROS',
              NAV2: 'Nav2 target'}

MODE_DESCRIPTIONS = {
    BASELINE: ('Perfect perception, in-sim oracle sharing: every boid knows '
               'true positions. Use it to study swarm tactics without '
               'sensing limits.'),
    SENSOR_ROS: ('Each boid sees only through its own limited FOV and range '
                 'and shares sightings one hop over a ROS topic. Tests the '
                 'tactics under real sensing and comms limits.'),
    NAV2: ('As Sensor + ROS, but the target flees along Nav2-planned paths. '
           'Needs ~5-8 s warm-up while Nav2 activates; the arena is frozen '
           'until then.'),
    None: ('Custom combination of perception / sharing / evader. Pick a '
           'preset above or edit the restart-marked fields and Apply.'),
}

PREFERRED_EVADER = {BASELINE: 'smart', SENSOR_ROS: 'smart', NAV2: 'nav2'}
FALLBACK_EVADER = 'reactive'

# Settings that only take effect when the stack (or world) is rebuilt.
RESTART_KEYS = ('perception', 'sharing_mode', 'evader',
                'shared_sighting_qos_depth', 'num_agents', 'env', 'seed')

PERCEPTIONS = ('perfect', 'sensor')
SHARING_MODES = ('legacy', 'off', 'oracle', 'ros')
ENVS = ('custom', 'open', 'obstacle_field', 'pillar', 'zones', 'shrink')
# The panel always offers these, whatever target_controller reports.
CORE_EVADERS = ('reactive', 'adaptive')
ALL_EVADERS = ('reactive', 'adaptive', 'smart', 'nav2')

# Live parameters that live on every boid_controller / on target_controller.
# They are forwarded into a restarted stack so tuning survives a restart.
AGENT_LIVE = ('pursuit_strategy', 'w_pursuit', 'lead_time',
              'ring_radius_start', 'commit_distance', 'w_separation',
              'w_alignment', 'w_cohesion', 'safe_distance', 'sensing_radius',
              'radio_range', 'sighting_timeout')
TARGET_LIVE = ('target_speed_multiplier', 'target_omega_max')

# Round-start fairness the window (UI / stack_managed) turns on for every
# mode: the legacy spawn drops the target inside the swarm for about half
# the seeds (spawn.py). Headless and ui:=false keep the sim defaults
# (off / 0) so pre-registered experiments stay reproducible.
UI_SIM_DEFAULTS = {'spawn_safe': True, 'spawn_min_clearance': 6.0,
                   'capture_grace': 1.5}

NAV2_WARMUP_S = 6.0
MIN_AGENTS, MAX_AGENTS = 2, 24

# launch arguments pursuit.launch.py forwards to the sim in UI mode so the
# sim can build the very same stack the headless path would have launched.
STACK_LAUNCH_KEYS = ('num_agents', 'strategy', 'seed', 'evader', 'perception',
                     'sharing_mode', 'env', 'obstacles', 'game_mode',
                     'shared_sighting_qos_depth', 'relay_log_dir',
                     'nav2_config', 'warmup', 'pursuer_delay')


class ConfigError(ValueError):
    """An invalid or contradictory stack configuration."""


def available_evaders():
    """Evader brains the installed target_controller can build.

    Read from the controller's own registry, so `smart` shows up in the
    panel exactly when `TargetController.BRAINS` has it (and an unknown
    brain — which would crash target_controller at construction — can never
    be offered). Falls back to the always-present pair when ROS is not
    importable (unit tests, CI)."""
    try:
        from .target_controller_node import TargetController
        names = tuple(TargetController.BRAINS)
    except Exception:                       # no rclpy / import failure
        return CORE_EVADERS
    return tuple(e for e in ALL_EVADERS if e in names) or CORE_EVADERS


def resolve_evader(preferred, available):
    """`preferred` if the controller has it, else the reactive fallback."""
    return preferred if preferred in available else FALLBACK_EVADER


def mode_preset(mode, available=ALL_EVADERS):
    """perception / sharing_mode / evader a mode stands for."""
    if mode not in MODES:
        raise ConfigError(f'unknown mode {mode!r}; expected {MODES}')
    return {
        BASELINE: {'perception': 'perfect', 'sharing_mode': 'legacy'},
        SENSOR_ROS: {'perception': 'sensor', 'sharing_mode': 'ros'},
        NAV2: {'perception': 'sensor', 'sharing_mode': 'ros'},
    }[mode] | {'evader': resolve_evader(PREFERRED_EVADER[mode], available)}


def mode_of(cfg):
    """Which preset `cfg` is, or None for a custom mix. smart/reactive/
    adaptive all count as the same mode: they differ in the brain, not in
    what the swarm may know."""
    nav2 = cfg.get('evader') == 'nav2'
    key = (cfg.get('perception'), cfg.get('sharing_mode'), nav2)
    return {('perfect', 'legacy', False): BASELINE,
            ('sensor', 'ros', False): SENSOR_ROS,
            ('sensor', 'ros', True): NAV2}.get(key)


def validate(cfg, available=ALL_EVADERS):
    """List of human-readable problems (empty = launchable)."""
    errs = []
    if cfg.get('perception') not in PERCEPTIONS:
        errs.append(f"perception must be one of {PERCEPTIONS}")
    if cfg.get('sharing_mode') not in SHARING_MODES:
        errs.append(f"sharing_mode must be one of {SHARING_MODES}")
    if cfg.get('sharing_mode') == 'ros' and cfg.get('perception') != 'sensor':
        errs.append('sharing_mode=ros requires perception=sensor')
    if cfg.get('evader') not in available:
        errs.append(f"evader '{cfg.get('evader')}' is not available "
                    f"(target_controller offers {'|'.join(available)})")
    if cfg.get('env') not in ENVS:
        errs.append(f"env must be one of {ENVS}")
    try:
        n = int(cfg.get('num_agents'))
        if not MIN_AGENTS <= n <= MAX_AGENTS:
            errs.append(f'num_agents must be {MIN_AGENTS}..{MAX_AGENTS}')
    except (TypeError, ValueError):
        errs.append('num_agents must be an integer')
    try:
        if int(cfg.get('shared_sighting_qos_depth', 10)) < 1:
            errs.append('shared_sighting_qos_depth must be >= 1')
    except (TypeError, ValueError):
        errs.append('shared_sighting_qos_depth must be an integer')
    return errs


def effective_warmup(cfg):
    """Seconds target+boid controllers are held back after the stack
    starts: the launch value if given, else enough for Nav2 to activate."""
    w = float(cfg.get('warmup') or 0.0)
    if w <= 0.0 and cfg.get('evader') == 'nav2':
        return NAV2_WARMUP_S
    return w


def stack_args(mode, overrides=None, base=None, available=ALL_EVADERS,
               live=None):
    """Launch arguments (str -> str) for swarm_stack.launch.py.

    `base`       defaults, e.g. the launch arguments the run started with
    `mode`       preset applied over `base` (None = leave base as is)
    `overrides`  explicit values, applied last (they win over the preset)
    `live`       live agent/target parameters to carry into the new stack

    Raises ConfigError when the merged result is contradictory."""
    cfg = {'num_agents': 8, 'strategy': 'auto', 'seed': 7,
           'evader': FALLBACK_EVADER, 'perception': 'perfect',
           'sharing_mode': 'legacy', 'env': 'custom', 'obstacles': '',
           'game_mode': 'ai', 'shared_sighting_qos_depth': '',
           'relay_log_dir': '', 'nav2_config': 'nav2_target.yaml',
           'warmup': 0.0, 'pursuer_delay': 0.0}
    cfg.update(base or {})
    if mode is not None:
        cfg.update(mode_preset(mode, available))
    cfg.update(overrides or {})
    if cfg['shared_sighting_qos_depth'] == '':
        probe = dict(cfg, shared_sighting_qos_depth=10)
    else:
        probe = cfg
    errs = validate(probe, available)
    if errs:
        raise ConfigError('; '.join(errs))
    live = dict(live or {})
    agent = {k: live[k] for k in AGENT_LIVE if k in live}
    target = {k: live[k] for k in TARGET_LIVE if k in live}
    args = {k: str(cfg[k]) for k in STACK_LAUNCH_KEYS}
    args['warmup'] = repr(effective_warmup(cfg))
    args['agent_params'] = json.dumps(agent, sort_keys=True)
    args['target_params'] = json.dumps(target, sort_keys=True)
    return args


def stack_command(args, launch_file='swarm_stack.launch.py'):
    """argv for `ros2 launch` (no shell: values are passed verbatim).
    Empty values are left out: `ros2 launch` rejects `key:=` as malformed,
    and an empty value means "use the launch default" anyway."""
    return (['ros2', 'launch', 'boids_swarm', launch_file]
            + [f'{k}:={v}' for k, v in args.items() if v != ''])


def expected_nodes(cfg):
    """Fully-qualified node names that must exist for the stack to be up."""
    names = [f"/agent{i}/boid_controller"
             for i in range(int(cfg['num_agents']))]
    if cfg.get('game_mode', 'ai') == 'ai':
        names.append('/target_controller')
    return names


def parse_mode_request(text, available=ALL_EVADERS):
    """Payload of `/ui/mode_request` -> overrides dict for the sim.

    Accepts a bare mode name (``sensor_ros``), or JSON
    ``{"mode": "nav2", "overrides": {"num_agents": 6}}`` /
    ``{"overrides": {...}}`` (no mode). Raises ConfigError on garbage."""
    text = (text or '').strip()
    if not text:
        raise ConfigError('empty mode request')
    mode, extra = None, {}
    if text.startswith('{'):
        try:
            doc = json.loads(text)
        except ValueError as exc:
            raise ConfigError(f'bad JSON: {exc}') from exc
        mode, extra = doc.get('mode'), dict(doc.get('overrides') or {})
    else:
        mode = text
    out = {}
    if mode is not None:
        out.update(mode_preset(mode, available))
    out.update(extra)
    unknown = set(out) - set(RESTART_KEYS)
    if unknown:
        raise ConfigError(f'cannot set {sorted(unknown)} through a request')
    return out
