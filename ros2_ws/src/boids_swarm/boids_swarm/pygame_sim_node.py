"""pygame_sim_node — THE WORLD (SDD v3 §1).

Sole owner of world state: integrates non-holonomic/holonomic physics for
all agents + target, detects capture, scores, renders, publishes poses.
Controllers talk to it only via ROS topics (pose out, cmd_vel in).

Single-threaded pygame + rclpy loop per §1.5. Physics uses a FIXED
timestep (1/fps) so seeded runs are comparable (§7.4 determinism);
`headless:=true` skips rendering and runs unthrottled (fast-forward).
"""

import atexit
import math
import os
import queue
import random
import signal
import time as _time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from boids_swarm_msgs.msg import EpisodeState, TargetSighting
from rclpy.parameter import Parameter as RclParameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
from turtlesim.msg import Pose

from .geometry import clamp, wrap_angle
from .game import (Scoreboard, TagHealth, capture_escape_blocked,
                   capture_hull)
from . import perception
from . import comms
from . import stack_config
from . import stack_supervisor
from . import ui
from .behaviors.pursuit import STRATEGIES
from .launch_util import obstacles_for_env
from .param_bridge import ParamBridge
from .ui_state import PanelState, status_view
from .world_gen import WorldGenerator

BG = (16, 20, 32)
ARENA = (90, 100, 120)
BOID = (80, 200, 255)
TARGET = (255, 90, 70)
OBSTACLE = (110, 110, 118)
HUD = (230, 235, 240)
TRAIL_BOID = (40, 90, 120)
TRAIL_TGT = (140, 60, 50)
BANNER_SEC = 2.0


class Entity:
    __slots__ = ('x', 'y', 'theta', 'v', 'w', 'cmd', 'trail')

    def __init__(self):
        self.x = self.y = self.theta = 0.0
        self.v = self.w = 0.0            # applied (published) velocities
        self.cmd = (0.0, 0.0)            # latest cmd_vel
        self.trail = []


class Nav2Probe:
    """True once Nav2's lifecycle manager answers `is_active` with success:
    the planner/controller servers are up, so the chase can start."""

    SERVICE = '/lifecycle_manager_target/is_active'

    def __init__(self, node):
        self.node = node
        self.client = node.create_client(Trigger, self.SERVICE)
        self.fut, self.t, self.done = None, 0.0, False

    def ready(self):
        if self.done:
            return True
        now = _time.monotonic()
        if self.fut is None:
            if self.client.service_is_ready():
                self.fut, self.t = self.client.call_async(
                    Trigger.Request()), now
            return False
        if self.fut.done():
            try:
                self.done = bool(self.fut.result().success)
            except Exception:
                self.done = False
            self.fut = None
        elif now - self.t > 2.0:
            self.fut = None                      # lost reply: ask again
        return self.done


class PygameSimNode(Node):

    def __init__(self):
        super().__init__('pygame_sim')
        # --- world / render params (§5) ---
        self.declare_parameter('num_agents', 12)
        self.declare_parameter('world_size', 20.0)
        self.declare_parameter('window_px', 800)
        self.declare_parameter('fps', 60)
        self.declare_parameter('headless', False)
        self.declare_parameter('motion_model', 'nonholonomic')
        self.declare_parameter('agent_max_speed', 2.0)
        self.declare_parameter('agent_omega_max', 6.0)
        self.declare_parameter('target_enabled', True)
        self.declare_parameter('target_speed_multiplier', 2.0)
        self.declare_parameter('target_omega_max', 2.5)   # the balance knob
        self.declare_parameter('target_stamina_enabled', False)
        self.declare_parameter('target_stamina_drain', 0.25)  # /s sprinting
        self.declare_parameter('target_stamina_regen', 0.35)
        self.declare_parameter('game_mode', 'ai')         # ai | human
        self.declare_parameter('capture_mode', 'hull')    # hull|escape_blocked|tag
        self.declare_parameter('capture_k', 3)
        self.declare_parameter('d_capture', 1.5)
        self.declare_parameter('episode_time_limit', 90.0)  # 0 = none
        self.declare_parameter('auto_reset', True)
        self.declare_parameter('episodes_max', 0)         # 0 = endless
        self.declare_parameter('obstacles', [0.0])        # flat [x,y,r,...]
        # --- procedural environments (v4 Part C, M13) ---
        # env_type != 'custom' derives obstacles/zones/shrink from the seed
        # (WorldGenerator), overriding the obstacles param.
        self.declare_parameter('env_type', 'custom')
        self.declare_parameter('shrink_rate', 0.0)        # units/s bounds inset
        self.declare_parameter('shrink_min', 4.0)         # stop shrinking here
        self.declare_parameter('zones', [0.0])            # flat [x,y,r,kind,...]
        self.declare_parameter('render_trails', False)
        self.declare_parameter('agent_radius_px', 6)
        # Physical collision radius per entity (v4). A larger target can't
        # squeeze through the wall-gap chokepoints that boids slip through
        # (SDD C.1: "boids fit but the larger target must detour").
        self.declare_parameter('target_body_radius', 0.7)
        self.declare_parameter('seed', 7)
        self.declare_parameter('screenshot_dir', '')   # '' = off
        self.declare_parameter('screenshot_period', 10.0)  # sim-seconds
        self.declare_parameter('pose_rate_hz', 30.0)   # pose publish rate
        # Fast-forward multiplier for headless benchmarking (§7.4). Bounded
        # (not "as fast as possible") so controller processes can still
        # keep up with their sim-time control rates.
        self.declare_parameter('time_scale', 4.0)
        # --- perception / sensor model (v4 Part A) ---
        # 'perfect' = v3 baseline (broadcast ground truth); 'sensor' = the
        # synthesized per-agent detection model. Ablate components with the
        # individual knobs below.
        self.declare_parameter('perception_mode', 'perfect')
        self.declare_parameter('fov', math.pi)             # half-angle; pi=omni
        self.declare_parameter('sensor_range', 8.0)
        self.declare_parameter('occlusion_enabled', True)
        self.declare_parameter('agent_body_radius', 0.25)
        self.declare_parameter('range_sigma', 0.02)        # * range (m/m)
        self.declare_parameter('bearing_sigma', 0.03)      # rad
        self.declare_parameter('p_miss', 0.05)             # * range/sensor_range
        self.declare_parameter('emit_ids', True)
        # information sharing (v4 A.6 / M10): a sighting propagates over a
        # range-limited mesh so the swarm collectively tracks the target.
        self.declare_parameter('comms_enabled', True)
        self.declare_parameter('comm_range', 8.0)
        self.declare_parameter('comm_jitter', 0.15)        # relay noise
        self.declare_parameter('sharing_mode', 'legacy')
        self.declare_parameter('radio_range', 8.0)
        self.declare_parameter('oracle_max_hops', 0)   # 0 = unlimited
        self.declare_parameter('relay_log_dir', '')
        self.declare_parameter('sighting_valid_for_sec', 0.6)
        # --- in-window control panel (v5 M15) ---
        self.declare_parameter('ui_enabled', True)
        self.declare_parameter('panel_px', 360)
        # Display-only name of the environment. env_type is forced to
        # 'custom' when the launch pre-generates the layout, so it cannot
        # be used to tell the user which env they actually asked for.
        self.declare_parameter('env_label', '')
        # Mirrors of parameters that actually LIVE on other nodes
        # (boid_controller x N, target_controller). The panel is immediate
        # mode and needs a value to draw every frame; querying N remote
        # nodes per frame would be absurd, so the sim keeps a local copy and
        # ParamBridge writes both sides on every edit. params.yaml uses the
        # `/**` wildcard, so these pick up the same defaults the real owners
        # do — and pursuit.launch.py forwards the launch args here too.
        self.declare_parameter('pursuit_strategy', 'auto')
        self.declare_parameter('w_pursuit', 2.0)
        self.declare_parameter('commit_distance', 2.5)
        self.declare_parameter('ring_radius_start', 3.0)
        self.declare_parameter('evader', 'reactive')
        self.declare_parameter('w_separation', 1.5)
        self.declare_parameter('w_alignment', 0.5)
        self.declare_parameter('w_cohesion', 0.5)
        self.declare_parameter('safe_distance', 1.2)
        self.declare_parameter('sensing_radius', 4.0)
        self.declare_parameter('lead_time', 1.0)
        self.declare_parameter('sighting_timeout', 0.6)
        # Mode switching (stack work). pursuit.launch.py sets stack_managed
        # when the panel is on: the sim then starts the controllers /
        # target / Nav2 itself (StackSupervisor) and can restart them with
        # the window open. stack_launch_args = the launch arguments of that
        # first stack, as JSON (see stack_config.STACK_LAUNCH_KEYS).
        self.declare_parameter('stack_managed', False)
        self.declare_parameter('stack_launch_args', '')
        self.add_on_set_parameters_callback(self._on_params)

        self.n = int(self.get_parameter('num_agents').value)
        self.world = float(self.get_parameter('world_size').value)
        self.fps = int(self.get_parameter('fps').value)
        self.headless = bool(self.get_parameter('headless').value)
        self.motion_model = self.get_parameter('motion_model').value
        self.target_enabled = bool(self.get_parameter('target_enabled').value)
        self.game_mode = self.get_parameter('game_mode').value
        self.rng = random.Random(int(self.get_parameter('seed').value))

        # Procedural layout (v4 M13): env_type derives the world from the
        # seed so strategy benchmarks stay fair; 'custom' uses the params.
        env_type = self.env_type = self.get_parameter('env_type').value
        self.shrink_rate = float(self.get_parameter('shrink_rate').value)
        if env_type != 'custom':
            layout = WorldGenerator(
                int(self.get_parameter('seed').value), self.world
            ).generate(env_type)
            self.obstacles = list(layout.obstacles)
            self.zones = list(layout.zones)
            if layout.shrink_rate > 0.0:
                self.shrink_rate = layout.shrink_rate
            self.get_logger().info(
                f'env={env_type} seed={self.get_parameter("seed").value} '
                f'obstacles={len(self.obstacles)} zones={len(self.zones)} '
                f'shrink_rate={self.shrink_rate}')
        else:
            self.obstacles = self._parse_obstacles(
                self.get_parameter('obstacles').value)
            self.zones = self._parse_zones(self.get_parameter('zones').value)
        # Live playable bounds (shrink from full arena inward, v4 C.4)
        self.cur_min = [0.0, 0.0]
        self.cur_max = [self.world, self.world]

        self.body_r = 0.15                       # boid physical radius
        self.target_body_r = float(
            self.get_parameter('target_body_radius').value)
        self.running = True
        self.paused = False                       # panel PAUSE button
        self.keys = None                          # set by run loop (human mode)
        self._next_shot = float(self.get_parameter('screenshot_period').value)
        # A panel needs a window and a mouse, so it is off when headless.
        self.ui_on = (bool(self.get_parameter('ui_enabled').value)
                      and not self.headless)
        self.panel = None                         # built in setup_display

        # --- entities ---
        self.agents = [Entity() for _ in range(self.n)]
        self.target = Entity()
        self.stamina = 1.0
        self.tag_hp = TagHealth()
        self.score = Scoreboard()
        self.state = 'running'                    # running | banner
        self.banner_t = 0.0
        self.banner_text = ''
        self.n_close = 0

        # --- ROS wiring (§1.4) ---
        # The sim owns time: it publishes /clock so controllers (running
        # with use_sim_time) stay in step even when headless fast-forward
        # runs physics much faster than wall time (§7.4).
        self.sim_time = 0.0
        self._episode_generation = 0
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        # Aggregated swarm poses on ONE topic (v2 §10.2 scaling option):
        # N controllers x 1 subscription instead of N x N. Layout:
        # [N, x0, y0, theta0, v0, w0, x1, ...]. Per-agent Pose topics are
        # kept for debugging/echo; controllers subscribe to the aggregate.
        self.swarm_pub = self.create_publisher(
            Float32MultiArray, '/swarm/poses', 10)
        # live arena bounds, so controllers track a shrinking wall (v4 C.4)
        self.bounds_pub = self.create_publisher(
            Float32MultiArray, '/arena/bounds', 10)
        # Per-agent synthesized detections (v4 A.4). Each controller
        # subscribes to only its own — fewer subs than the broadcast AND
        # realistic. Published only in 'sensor' mode.
        self.perception_mode = self.get_parameter('perception_mode').value
        epoch_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.episode_pub = self.create_publisher(EpisodeState,
            '/simulation/episode_state', epoch_qos)
        self.observation_pub = self.create_publisher(String,
            '/swarm/observations', 10)
        self.sighting_pubs, self.det_pubs, self.pose_pubs = [], [], []
        self._cmd_subs = []
        self.sighting_sequences = [1] * self.n
        self.percept_rng = random.Random(
            int(self.get_parameter('seed').value) + 104729)
        self.frame = 0
        self.pose_div = max(1, round(
            self.fps / float(self.get_parameter('pose_rate_hz').value)))
        self._ensure_agent_io(self.n)
        if self.target_enabled:
            self.target_pose_pub = self.create_publisher(
                Pose, '/target/pose', 10)
            self.create_subscription(
                Twist, '/target/cmd_vel',
                lambda m: self._on_cmd(m, self.target), 1)

        # Panel edits are enqueued on the render thread and flushed here, on
        # the executor thread, so all rclpy work stays single-threaded.
        self.bridge = None
        if self.ui_on:
            self.bridge = ParamBridge(self, self.n, self.target_enabled)
            self.create_timer(0.05, self.bridge.pump)

        # --- restartable stack (modes) ---
        self.managed = bool(self.get_parameter('stack_managed').value) \
            and self.ui_on
        self.available_evaders = stack_config.available_evaders()
        self.supervisor = stack_supervisor.StackSupervisor()
        self._stack_base = {}
        raw = self.get_parameter('stack_launch_args').value
        if raw:
            self._stack_base = json.loads(raw)
        self.pstate = PanelState(self._initial_cfg(), self.available_evaders,
                                 managed=self.managed)
        self._requests = queue.Queue()        # mode requests from topics
        self._last_sup_state = None
        self._status_sent = (None, 0.0)
        self._notice = ('', 0.0)
        self.hold = self.managed            # frozen until the stack is up
        status_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(
            String, '/ui/stack_status', status_qos)
        # Automation / scripting hook: same effect as clicking a mode
        # button (bare mode name, or JSON with overrides).
        self.create_subscription(
            String, '/ui/mode_request',
            lambda m: self._requests.put(m.data), 10)

        self._spawn_all()
        self.score.start_episode()
        self._publish_episode_state()
        self.get_logger().info(
            f'world up: N={self.n}, {self.world}x{self.world}, '
            f'model={self.motion_model}, target={self.target_enabled} '
            f'(mode={self.game_mode}), obstacles={len(self.obstacles)}, '
            f'headless={self.headless}')

    # ------------------------------------------------------------------ util
    @staticmethod
    def _parse_obstacles(flat):
        flat = [float(v) for v in (flat or [])]
        if len(flat) < 3:
            return []
        return [(flat[i], flat[i + 1], flat[i + 2])
                for i in range(0, len(flat) - 2, 3)]

    @staticmethod
    def _parse_zones(flat):
        """flat [x, y, r, kind, ...] with kind 0.0=capture, 1.0=slow."""
        flat = [float(v) for v in (flat or [])]
        if len(flat) < 4:
            return []
        kinds = {0.0: 'capture', 1.0: 'slow'}
        return [(flat[i], flat[i + 1], flat[i + 2],
                 kinds.get(flat[i + 3], 'capture'))
                for i in range(0, len(flat) - 3, 4)]

    def _on_params(self, params):
        for p in params:
            if p.name == 'obstacles':
                self.obstacles = self._parse_obstacles(p.value)
            elif p.name == 'game_mode':
                self.game_mode = p.value
            elif p.name == 'perception_mode':
                self.perception_mode = p.value
        return SetParametersResult(successful=True)

    def _p(self, name):
        return self.get_parameter(name).value

    def _initial_cfg(self):
        """The restart-needing settings the first stack was launched with
        (launch arguments when managed, else this node's own parameters)."""
        b = self._stack_base
        qos = b.get('shared_sighting_qos_depth', '')
        return {
            'perception': b.get('perception', self.perception_mode),
            'sharing_mode': b.get('sharing_mode', self._p('sharing_mode')),
            'evader': b.get('evader', self._p('evader')),
            'shared_sighting_qos_depth': int(qos) if qos != '' else 10,
            'num_agents': self.n,
            'env': b.get('env') or self._p('env_label') or self.env_type,
            'seed': int(self._p('seed')),
        }

    def _ensure_agent_io(self, n):
        """Publishers/subscriptions for agents 0..n-1. Created once and
        kept when the swarm shrinks (rebuild_world), so a world rebuild
        never has to destroy ROS entities under a spinning executor."""
        for i in range(len(self.pose_pubs), n):
            self.sighting_pubs.append(self.create_publisher(
                TargetSighting, f'/agent{i}/local_target_sighting', 10))
            self.det_pubs.append(self.create_publisher(
                Float32MultiArray, f'/agent{i}/detections', 10))
            self.pose_pubs.append(
                self.create_publisher(Pose, f'/agent{i}/pose', 10))
            # depth=1: only the LATEST command matters — a deeper queue
            # holds stale commands and turns into actuation delay.
            self._cmd_subs.append(self.create_subscription(
                Twist, f'/agent{i}/cmd_vel',
                lambda m, k=i: self._on_agent_cmd(m, k), 1))

    def _on_agent_cmd(self, msg, k):
        agents = self.agents
        if k < len(agents):
            self._on_cmd(msg, agents[k])

    def _on_cmd(self, msg: Twist, ent: Entity):
        if self.motion_model == 'holonomic':
            ent.cmd = (msg.linear.x, msg.linear.y)
        else:
            ent.cmd = (msg.linear.x, msg.angular.z)

    # ----------------------------------------------------------------- spawn
    def _free_pos(self, margin=1.5, clear_of=(), min_dist=0.0, tries=200):
        for _ in range(tries):
            x = self.rng.uniform(margin, self.world - margin)
            y = self.rng.uniform(margin, self.world - margin)
            if any(math.hypot(x - ox, y - oy) < r + 0.8
                   for (ox, oy, r) in self.obstacles):
                continue
            if any(math.hypot(x - cx, y - cy) < min_dist
                   for (cx, cy) in clear_of):
                continue
            return x, y
        return self.world / 2.0, self.world / 2.0

    def _spawn_all(self):
        self.cur_min = [0.0, 0.0]                   # reset shrinking bounds
        self.cur_max = [self.world, self.world]
        for a in self.agents:
            a.x, a.y = self._free_pos()
            a.theta = self.rng.uniform(-math.pi, math.pi)
            a.v = a.w = 0.0
            a.cmd = (0.0, 0.0)
            a.trail.clear()
        if self.target_enabled:
            swarm = [(a.x, a.y) for a in self.agents]
            t = self.target
            t.x, t.y = self._free_pos(clear_of=swarm, min_dist=6.0)
            t.theta = self.rng.uniform(-math.pi, math.pi)
            t.v = t.w = 0.0
            t.cmd = (0.0, 0.0)
            t.trail.clear()
            self.stamina = 1.0
            self.tag_hp.reset()

    def rebuild_world(self, n, env, seed):
        """Swap swarm size / environment / seed under the open window.

        Same rules as launch time: a procedural env is generated from
        (seed, world_size) with WorldGenerator, 'custom' keeps the launch's
        obstacle string; the controllers of the NEW stack receive the very
        same layout because pursuit's `obstacles_for_env` is the shared
        source. Called on the main thread, with the old stack stopped or
        stopping (its poses are ignored: the lists below are replaced)."""
        self.n = int(n)
        self._ensure_agent_io(self.n)
        self.agents = [Entity() for _ in range(self.n)]
        self.sighting_sequences = [1] * self.n
        self.rng = random.Random(int(seed))
        self.percept_rng = random.Random(int(seed) + 104729)
        if env != 'custom':
            layout = WorldGenerator(int(seed), self.world).generate(env)
            self.obstacles = list(layout.obstacles)
            self.zones = list(layout.zones)
            self.shrink_rate = layout.shrink_rate if layout.shrink_rate > 0 \
                else 0.0
        else:
            flat = obstacles_for_env('custom', int(seed), self.world,
                                     self._stack_base.get('obstacles', ''))
            self.obstacles = self._parse_obstacles(flat)
            self.zones = []
            self.shrink_rate = 0.0
        self.env_type = env
        if self.bridge is not None:
            self.bridge.set_n(self.n)
        self.set_parameters([
            RclParameter('num_agents', value=self.n),
            RclParameter('seed', value=int(seed)),
            RclParameter('env_label', value=env)])

    def _publish_episode_state(self):
        msg = EpisodeState()
        msg.header.frame_id = 'world'
        msg.header.stamp.sec = int(self.sim_time)
        msg.header.stamp.nanosec = int((self.sim_time % 1.0) * 1e9)
        self._episode_generation += 1
        msg.episode_id = self._episode_generation
        self.episode_pub.publish(msg)
        self.sighting_sequences = [1] * self.n

    # --------------------------------------------------------------- physics
    def _integrate(self, ent: Entity, dt, v_max, w_max):
        br = self.target_body_r if ent is self.target else self.body_r
        if self.motion_model == 'holonomic':
            vx, vy = ent.cmd
            sp = math.hypot(vx, vy)
            if sp > v_max and sp > 1e-9:
                vx, vy = vx / sp * v_max, vy / sp * v_max
                sp = v_max
            ent.x += vx * dt
            ent.y += vy * dt
            if sp > 1e-3:
                ent.theta = math.atan2(vy, vx)
            ent.v, ent.w = sp, 0.0
        else:
            v = clamp(ent.cmd[0], 0.0, v_max)      # no reversing (§2)
            w = clamp(ent.cmd[1], -w_max, w_max)
            ent.theta = wrap_angle(ent.theta + w * dt)
            ent.x += v * math.cos(ent.theta) * dt
            ent.y += v * math.sin(ent.theta) * dt
            ent.v, ent.w = v, w
        # walls (use the LIVE bounds so a shrinking arena squeezes, v4 C.4)
        ent.x = clamp(ent.x, self.cur_min[0] + br, self.cur_max[0] - br)
        ent.y = clamp(ent.y, self.cur_min[1] + br, self.cur_max[1] - br)
        # obstacles: push out to surface (blocking, not deadly)
        for (cx, cy, r) in self.obstacles:
            dx, dy = ent.x - cx, ent.y - cy
            d = math.hypot(dx, dy)
            keep = r + br
            if d < keep:
                if d < 1e-9:
                    dx, dy, d = 1.0, 0.0, 1.0
                ent.x = cx + dx / d * keep
                ent.y = cy + dy / d * keep

    def _target_caps(self, dt):
        base = self._p('agent_max_speed')
        mult = self._p('target_speed_multiplier')
        v_cap = base * mult
        if self._p('target_stamina_enabled'):
            sprinting = self.target.cmd[0] > base + 1e-6 \
                if self.motion_model != 'holonomic' \
                else math.hypot(*self.target.cmd) > base + 1e-6
            if sprinting and self.stamina > 0.0:
                self.stamina = max(0.0, self.stamina
                                   - self._p('target_stamina_drain') * dt)
            else:
                self.stamina = min(1.0, self.stamina
                                   + self._p('target_stamina_regen') * dt)
            if self.stamina <= 1e-6:
                v_cap = base                       # exhausted: no sprint
        # slow zone / tar pit nullifies the 2x advantage (v4 C.2)
        if self._in_zone(self.target, 'slow'):
            v_cap = min(v_cap, base)
        return v_cap, self._p('target_omega_max')

    def _in_zone(self, ent, kind):
        for (zx, zy, zr, zk) in self.zones:
            if zk == kind and math.hypot(ent.x - zx, ent.y - zy) < zr:
                return True
        return False

    def _shrink_step(self, dt):
        """Contract the playable bounds toward centre (v4 C.4). Perimeter
        circling becomes impossible once the loop is shorter than the
        target's turn radius, so every episode terminates."""
        if self.shrink_rate <= 0.0:
            return
        lo = self._p('shrink_min')
        inset = self.shrink_rate * dt
        cx = cy = self.world * 0.5
        if self.cur_max[0] - self.cur_min[0] > lo:
            self.cur_min[0] = min(self.cur_min[0] + inset, cx - lo / 2)
            self.cur_max[0] = max(self.cur_max[0] - inset, cx + lo / 2)
        if self.cur_max[1] - self.cur_min[1] > lo:
            self.cur_min[1] = min(self.cur_min[1] + inset, cy - lo / 2)
            self.cur_max[1] = max(self.cur_max[1] - inset, cy + lo / 2)

    def _human_drive(self):
        """Arrow keys drive the target directly (game_mode=human, §4.7)."""
        import pygame
        if self.keys is None:
            return
        v_cap = self._p('agent_max_speed') * self._p('target_speed_multiplier')
        w_cap = self._p('target_omega_max')
        v = 0.0
        if self.keys[pygame.K_UP]:
            v = v_cap
        elif self.keys[pygame.K_DOWN]:
            v = 0.3 * self._p('agent_max_speed')
        w = 0.0
        if self.keys[pygame.K_LEFT]:
            w = w_cap
        if self.keys[pygame.K_RIGHT]:
            w = -w_cap
        self.target.cmd = (v, w)

    def step_physics(self, dt):
        if self.state != 'running':
            return
        self._shrink_step(dt)                       # v4 C.4 moving walls
        v_max = self._p('agent_max_speed')
        w_max = self._p('agent_omega_max')
        for a in self.agents:
            self._integrate(a, dt, v_max, w_max)
        if self.target_enabled:
            if self.game_mode == 'human':
                self._human_drive()
            tv, tw = self._target_caps(dt)
            self._integrate(self.target, dt, tv, tw)
        if self._p('render_trails'):
            for e in self.agents + ([self.target] if self.target_enabled else []):
                e.trail.append((e.x, e.y))
                if len(e.trail) > 150:
                    e.trail.pop(0)

    # ---------------------------------------------------------- game logic
    def check_capture_and_score(self, dt):
        if self.state == 'banner':
            self.banner_t -= dt
            if self.banner_t <= 0.0:
                em = int(self._p('episodes_max'))
                done = bool(em) and self.score.episode >= em
                if self._p('auto_reset') and not done:
                    self._spawn_all()
                    self.score.start_episode()
                    self._publish_episode_state()
                    self.state = 'running'
                else:
                    # Either the requested episode count is complete, or
                    # auto_reset is off and there is nothing left to run —
                    # end the run instead of sitting on the banner forever
                    # (the old code only exited via the auto_reset path).
                    self._final_summary()
                    self.running = False
            return

        boid_xy = [(a.x, a.y) for a in self.agents]
        self.score.tick(dt, boid_xy)
        if not self.target_enabled:
            return

        t_xy = (self.target.x, self.target.y)
        mode = self._p('capture_mode')
        d_cap = self._p('d_capture')
        captured = False
        if mode == 'hull':
            captured, self.n_close = capture_hull(
                t_xy, boid_xy, int(self._p('capture_k')), d_cap)
        elif mode == 'escape_blocked':
            captured, self.n_close = capture_escape_blocked(
                t_xy, boid_xy, d_cap)
        elif mode == 'tag':
            captured, self.n_close = self.tag_hp.update(
                t_xy, boid_xy, d_cap, dt)

        # capture zone / tar pit: herd the target in for a direct win (C.2)
        if self._in_zone(self.target, 'capture'):
            captured = True

        limit = self._p('episode_time_limit')
        if captured:
            self._end_episode('captured')
        elif limit > 0.0 and self.score.t_episode >= limit:
            self._end_episode('timeout')

    def _end_episode(self, result):
        self.score.end_episode(result == 'captured')
        line = self.score.summary_line(result)
        self.get_logger().info(line)
        print(line, flush=True)                    # §7.4 metrics hook
        self.banner_text = ('CAPTURED in %.1fs' % self.score.t_episode
                            if result == 'captured' else 'TARGET ESCAPED')
        self.state = 'banner'
        self.banner_t = 0.0 if self.headless else BANNER_SEC

    def _final_summary(self):
        s = self.score
        avg = (sum(s.capture_times) / len(s.capture_times)
               if s.capture_times else float('nan'))
        # `episode` is now the count actually RUN: the terminal episode no
        # longer starts a new one before the summary fires.
        line = (f'SUMMARY episodes={s.episode} captures={s.captures} '
                f'timeouts={s.timeouts} avg_capture_t={avg:.2f}s')
        self.get_logger().info(line)
        print(line, flush=True)

    # -------------------------------------------------------------- ROS out
    def publish_states(self):
        clk = Clock()
        clk.clock.sec = int(self.sim_time)
        clk.clock.nanosec = int((self.sim_time % 1.0) * 1e9)
        self.clock_pub.publish(clk)
        self.frame += 1
        if self.frame % self.pose_div:
            return                     # poses at pose_rate_hz, not fps
        # /swarm/poses (aggregated ground truth): consumed by perfect-mode
        # boids and always by the target_controller (its perception stays
        # perfect in M8) + debug. /agentI/pose is each agent's own odometry.
        arr = Float32MultiArray()
        data = [float(self.n)]
        for a in self.agents:
            data += [a.x, a.y, a.theta, a.v, a.w]
        arr.data = [float(v) for v in data]
        self.swarm_pub.publish(arr)
        bmsg = Float32MultiArray()
        bmsg.data = [float(self.cur_min[0]), float(self.cur_min[1]),
                     float(self.cur_max[0]), float(self.cur_max[1])]
        self.bounds_pub.publish(bmsg)
        for i, a in enumerate(self.agents):
            self.pose_pubs[i].publish(self._pose_msg(a))
        if self.target_enabled:
            self.target_pose_pub.publish(self._pose_msg(self.target))

        if self.perception_mode == 'sensor':
            self._publish_detections()
        # M7 (SDD §6.4): additionally publish /target/odom, TF and /map here.

    def _sensor_cfg(self):
        return {
            'fov': self._p('fov'),
            'sensor_range': self._p('sensor_range'),
            'occlusion': self._p('occlusion_enabled'),
            'range_sigma': self._p('range_sigma'),
            'bearing_sigma': self._p('bearing_sigma'),
            'p_miss': self._p('p_miss'),
            'emit_ids': self._p('emit_ids'),
        }

    def _log_expected_receivers(self, i, sighting):
        """Phase 2 R-03: per-sighting set of agents inside radio_range of
        the sender at publish time (the per-receiver loss denominator).
        Written to a file (not a topic) so it cannot itself be lost."""
        if self._p('sharing_mode') != 'ros':
            return
        log_dir = self._p('relay_log_dir')
        if not log_dir:
            return
        if getattr(self, '_expected_log', None) is None:
            os.makedirs(log_dir, exist_ok=True)
            self._expected_log = open(
                os.path.join(log_dir, 'expected_receivers.jsonl'), 'a',
                buffering=1)
        positions = [(a.x, a.y) for a in self.agents]
        expected = comms.receivers_in_range(
            positions, i, self._p('radio_range'))
        self._expected_log.write(json.dumps(
            {'k': [int(sighting.episode_id), sighting.sender_id,
                   int(sighting.sequence)],
             't': self.sim_time,
             'exp': [f'agent{j}' for j in expected]}) + '\n')

    def _publish_detections(self):
        """Synthesize each agent's own detection stream (v4 A.2), then
        relay target sightings over the comm mesh (v4 A.6)."""
        cfg = self._sensor_cfg()
        mode = self._p('sharing_mode')
        if mode == 'legacy':
            mode = 'oracle' if self._p('comms_enabled') else 'off'
        body_r = self._p('agent_body_radius')
        # candidate id convention: 0..N-1 agents, N = target.
        base = [(i, a.x, a.y, a.theta, a.v, False)
                for i, a in enumerate(self.agents)]
        if self.target_enabled:
            t = self.target
            base.append((self.n, t.x, t.y, t.theta, t.v, True))
        bodies = [(a.x, a.y, body_r) for a in self.agents]
        if self.target_enabled:
            bodies.append((self.target.x, self.target.y, body_r))
        occ_static = list(self.obstacles)

        # pass 1: each agent's own (FOV/occlusion/noise/dropout) detections
        per_agent = []
        seers = [False] * self.n
        for i, a in enumerate(self.agents):
            observer = (a.x, a.y, a.theta)
            cands = [c for c in base if c[0] != i]
            occluders = occ_static + [b for j, b in enumerate(bodies)
                                      if j != i]
            dets = perception.compute_detections(
                observer, cands, occluders, cfg, self.percept_rng)
            per_agent.append(dets)
            seers[i] = any(d[4] >= 0.5 for d in dets)   # saw the target?
            target_det = next((d for d in dets if d[4] >= 0.5), None)
            if target_det is not None:
                sighting = perception.target_measurement_world(
                    observer, target_det, stamp=self.sim_time,
                    sequence=self.sighting_sequences[i],
                    episode_id=self._episode_generation, sender_id=f'agent{i}',
                    range_sigma=cfg['range_sigma'],
                    bearing_sigma=cfg['bearing_sigma'],
                    sensor_range=cfg['sensor_range'],
                    valid_for_sec=self._p('sighting_valid_for_sec'))
                self.sighting_sequences[i] += 1
                self.sighting_pubs[i].publish(sighting)
                self._log_expected_receivers(i, sighting)
                record = {'episode_id': sighting.episode_id,
                          'sender_id': sighting.sender_id,
                          'sequence': sighting.sequence, 'stamp': self.sim_time,
                          'raw_range': target_det[0],
                          'raw_bearing': target_det[1],
                          'target_position': [sighting.target_position.x,
                                              sighting.target_position.y],
                          'sender_position': [a.x, a.y],
                          'covariance_xy': list(sighting.covariance_xy)}
                self.observation_pub.publish(String(data=json.dumps(record)))

        # pass 2: mesh relay — informed agents that did NOT see the target
        # directly get a relayed sighting in their own frame.
        if (mode == 'oracle' and self.target_enabled and any(seers)):
            positions = [(a.x, a.y) for a in self.agents]
            informed = comms.propagate_sightings(
                positions, self._p('comm_range'), seers,
                max_hops=int(self._p('oracle_max_hops')))
            jitter = self._p('comm_jitter')
            tgt = (self.target.x, self.target.y, self.target.theta,
                   self.target.v)
            for i, a in enumerate(self.agents):
                if informed[i] and not seers[i]:
                    per_agent[i].append(perception.relayed_detection(
                        (a.x, a.y, a.theta), tgt, self.n, jitter,
                        self.percept_rng))

        for i in range(self.n):
            if mode == 'ros':
                per_agent[i] = [d for d in per_agent[i] if d[4] < 0.5]
            msg = Float32MultiArray()
            msg.data = perception.detections_to_flat(per_agent[i])
            self.det_pubs[i].publish(msg)

    @staticmethod
    def _pose_msg(e: Entity) -> Pose:
        m = Pose()
        m.x, m.y, m.theta = float(e.x), float(e.y), float(e.theta)
        m.linear_velocity, m.angular_velocity = float(e.v), float(e.w)
        return m

    # --------------------------------------------------------------- render
    def setup_display(self):
        import pygame
        if self.headless:
            os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
        pygame.init()
        px = int(self._p('window_px'))
        self.arena_px = px
        self.font = pygame.font.SysFont('monospace', 15)
        self.big_font = pygame.font.SysFont('monospace', 34, bold=True)
        self.fonts = {
            'body': pygame.font.SysFont('monospace', 15),
            'small': pygame.font.SysFont('monospace', 13),
            'tiny': pygame.font.SysFont('monospace', 11),
            'bold': pygame.font.SysFont('monospace', 13, bold=True),
        }
        panel_px = 0
        win_h = px
        if self.ui_on:
            panel_px = int(self._p('panel_px'))
            self.panel = ui.build_panel(
                strategies=tuple(STRATEGIES),
                capture_modes=('hull', 'escape_blocked', 'tag'),
                # only brains the installed target_controller can build
                # (it would crash at construction on an unknown one)
                evaders=self.available_evaders,
                envs=stack_config.ENVS, width=panel_px)
            # Lay out first: a tall panel decides the window height, so a
            # small window_px doesn't clip the bottom controls away.
            win_h = max(px, self.panel.layout(px, 0))
        self.screen = pygame.display.set_mode((px + panel_px, win_h))
        pygame.display.set_caption('boids_swarm — cooperative pursuit')
        self.scale = px / self.world

    def _to_px(self, x, y):
        return int(x * self.scale), int((self.world - y) * self.scale)

    # ------------------------------------------------------------ panel
    def _mode_label(self):
        m = self.pstate.mode
        return stack_config.MODE_LABELS.get(m, 'Custom')

    def _mode_description(self):
        text = stack_config.MODE_DESCRIPTIONS[self.pstate.mode]
        ev = self.pstate.applied.get('evader')
        if self.pstate.mode in (stack_config.BASELINE,
                                stack_config.SENSOR_ROS):
            text += f' Evader: {ev}.'
        return text

    def _status_view(self):
        notice = self._notice[0] if self._notice[1] > _time.monotonic() else ''
        return status_view(self.supervisor.status(), self.managed,
                           notice or self.pstate.notice
                           or (self.bridge.last_error if self.bridge
                               else ''), self._mode_label())

    def _ui_value(self, scope, key):
        """Live value for a panel row. AGENTS/TARGET rows read the local
        mirror (ParamBridge writes it back once the controllers confirm);
        RESTART rows read the staged-or-applied config; ACTION rows report
        toggle state and the header's dynamic text."""
        if scope == ui.ACTION:
            if key == 'toggle_pause':
                return self.paused
            if key == 'mode_bar':
                return {'current': self.pstate.mode,
                        'enabled': self.managed}
            if key == 'mode_desc':
                return self._mode_description()
            if key == 'status':
                return self._status_view()
            if key == 'apply':
                return self.pstate.pending_count
            if key == 'buttons':
                return {'toggle_pause': self.paused}
            return False
        if scope == ui.RESTART:
            return self.pstate.value(key)
        return self._p(key)

    def _notify(self, text, seconds=6.0):
        self._notice = (text, _time.monotonic() + seconds)

    def _ui_apply(self, change):
        if change is None:
            return
        kind, *rest = self.pstate.handle(change)
        if kind == 'stage':
            return
        if kind == 'notice':
            self._notify(rest[0])
            return
        if kind == 'restart':
            self.request_stack_change(rest[0])
            return
        if kind == 'action':
            self._do_action(rest[0])
            return
        scope, key, value = rest
        self.bridge.request(scope, key, value)

    def _do_action(self, key):
        if key == 'toggle_pause':
            self.paused = not self.paused
        elif key == 'reset_episode':
            # A manual do-over restarts the CURRENT episode rather than
            # counting a new one, so the panel can't inflate the score.
            self._spawn_all()
            self.score.restart_episode()
            self._publish_episode_state()
            self.state = 'running'

    # ------------------------------------------------------- stack / modes
    def _live_values(self):
        names = stack_config.AGENT_LIVE + stack_config.TARGET_LIVE
        return {k: self._p(k) for k in names}

    def _launch_args(self, cfg):
        """swarm_stack.launch.py arguments for `cfg` (ConfigError if bad)."""
        live = self._live_values()
        overrides = dict(cfg, strategy=live['pursuit_strategy'])
        return stack_config.stack_args(
            None, overrides=overrides, base=self._stack_base,
            available=self.available_evaders, live=live)

    def _make_probe(self, cfg):
        """Readiness of a freshly started stack: every controller service
        discoverable, plus Nav2's lifecycle manager reporting active."""
        ai = self._stack_base.get('game_mode', self.game_mode) == 'ai'
        nav2 = Nav2Probe(self) if cfg['evader'] == 'nav2' else None

        def probe():
            if not self.bridge.all_ready(include_target=ai):
                return False
            return nav2.ready() if nav2 is not None else True
        return probe

    def _launch_stack(self, cfg, first=False):
        args = self._launch_args(cfg)
        nav2 = cfg['evader'] == 'nav2'
        warm = float(args['warmup'])
        kw = dict(probe=self._make_probe(cfg),
                  settle_s=1.0 if nav2 else 0.8,
                  eta_s=warm + (4.0 if nav2 else 2.5),
                  label='nav2' if nav2 else self._mode_label())
        cmd = stack_config.stack_command(args)
        if first:
            self.supervisor.start(cmd, **kw)
        else:
            self.supervisor.switch(cmd, **kw)

    def start_initial_stack(self):
        """Called once the window is up: start the first stack from the
        launch arguments (the same ones the headless launch would use)."""
        if not self.managed:
            return
        try:
            self._launch_stack(self.pstate.applied, first=True)
        except stack_config.ConfigError as exc:
            self.supervisor.state = stack_supervisor.FAILED
            self.supervisor.detail = str(exc)

    def request_stack_change(self, overrides):
        """Rebuild the stack (and the world, if its shape changed) with the
        staged edits + `overrides`. Main thread only."""
        if not self.managed:
            self._notify('stack not managed by the sim: re-launch instead')
            return False
        cfg = self.pstate.target_cfg(overrides)
        errs = stack_config.validate(cfg, self.available_evaders)
        if errs:
            self._notify(errs[0])
            self.get_logger().warn(f'refused stack change: {errs}')
            return False
        old = self.pstate.applied
        world_changed = any(cfg[k] != old[k]
                            for k in ('num_agents', 'env', 'seed'))
        if world_changed:
            self.rebuild_world(cfg['num_agents'], cfg['env'], cfg['seed'])
        self.set_parameters([
            RclParameter('perception_mode', value=cfg['perception']),
            RclParameter('sharing_mode', value=cfg['sharing_mode']),
            RclParameter('evader', value=cfg['evader'])])
        self.bridge.flush()               # replies from the old stack: stale
        self.pstate.commit(cfg)
        self.score = Scoreboard()
        self.score.start_episode()
        self._spawn_all()
        self._publish_episode_state()
        self.state = 'running'
        self.hold = True
        try:
            self._launch_stack(cfg)
        except stack_config.ConfigError as exc:
            self._notify(str(exc))
            return False
        self.get_logger().info(
            f'stack -> {self._mode_label()} {cfg}')
        return True

    def _process_requests(self):
        while True:
            try:
                text = self._requests.get_nowait()
            except queue.Empty:
                return
            try:
                ov = stack_config.parse_mode_request(
                    text, self.available_evaders)
            except stack_config.ConfigError as exc:
                self._notify(f'bad mode request: {exc}')
                self.get_logger().warn(f'bad /ui/mode_request: {exc}')
                continue
            if self.managed:
                self.request_stack_change(ov)
            else:
                self._notify('stack not managed by the sim')

    def tick_stack(self):
        """Once per frame, main thread: requests, supervisor, hold, topic."""
        self._process_requests()
        if not self.managed:
            return
        self.supervisor.tick()
        # live brain swaps (reactive/adaptive/smart) change the mirror only
        self.pstate.applied['evader'] = self._p('evader')
        st = self.supervisor.state
        if st == stack_supervisor.RUNNING \
                and self._last_sup_state != stack_supervisor.RUNNING:
            # a fresh stack is up: start the episode from a clean slate
            self._spawn_all()
            self.score.restart_episode()
            self._publish_episode_state()
            self.state = 'running'
        self.hold = st != stack_supervisor.RUNNING
        self._last_sup_state = st
        self._publish_status(st)

    def _publish_status(self, st):
        now = _time.monotonic()
        last_st, last_t = self._status_sent
        if st == last_st and now - last_t < 1.0:
            return
        self._status_sent = (st, now)
        doc = dict(self.supervisor.status(), mode=self.pstate.mode,
                   applied=self.pstate.applied, hold=self.hold,
                   managed=self.managed)
        self.status_pub.publish(String(data=json.dumps(doc)))

    def handle_pygame_events(self):
        import pygame
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (
                    ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                self.running = False
            elif self.panel is None:
                continue
            elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                self._ui_apply(self.panel.mouse_down(ev.pos, self._ui_value))
            elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
                self.panel.mouse_up()
            elif ev.type == pygame.MOUSEMOTION:
                self._ui_apply(self.panel.mouse_move(ev.pos))
        self.keys = pygame.key.get_pressed()

    def _triangle(self, e: Entity, size_px, color):
        import pygame
        cx, cy = self._to_px(e.x, e.y)
        pts = []
        for ang in (0.0, 2.5, -2.5):               # arrowhead
            a = e.theta + ang
            r = size_px if ang == 0.0 else size_px * 0.75
            pts.append((cx + r * math.cos(a), cy - r * math.sin(a)))
        pygame.draw.polygon(self.screen, color, pts)

    def _draw_hold_overlay(self, pygame):
        """The world is frozen while the stack (re)starts; say why."""
        v = self._status_view()
        if getattr(self, '_veil', None) is None:     # built once: per-frame
            self._veil = pygame.Surface(              # 800x800 alpha is slow
                (self.arena_px, self.arena_px), pygame.SRCALPHA)
            self._veil.fill((10, 14, 24, 150))
        self.screen.blit(self._veil, (0, 0))
        for i, (txt, font, col) in enumerate((
                (v['text'], self.big_font, (255, 220, 90)),
                ('world paused until the controllers are up', self.font,
                 HUD))):
            img = font.render(txt[:46], True, col)
            self.screen.blit(img, (self.arena_px // 2 - img.get_width() // 2,
                                   self.arena_px // 2 - 30 + i * 44))

    def render(self):
        shot_dir = self._p('screenshot_dir')
        shot_due = bool(shot_dir) and self.sim_time >= self._next_shot
        if self.headless and not shot_due:
            return
        import pygame
        self.screen.fill(BG)
        # live (possibly shrinking) playable bounds
        bx0, by0 = self._to_px(self.cur_min[0], self.cur_max[1])
        pygame.draw.rect(
            self.screen, ARENA,
            (bx0, by0, (self.cur_max[0] - self.cur_min[0]) * self.scale,
             (self.cur_max[1] - self.cur_min[1]) * self.scale), 2)
        # zones (v4 C.2): capture = green, slow/tar = amber
        for (zx, zy, zr, zk) in self.zones:
            col = (60, 180, 90) if zk == 'capture' else (170, 130, 50)
            surf = pygame.Surface((int(2 * zr * self.scale),
                                   int(2 * zr * self.scale)), pygame.SRCALPHA)
            pygame.draw.circle(surf, (*col, 70),
                               (int(zr * self.scale), int(zr * self.scale)),
                               int(zr * self.scale))
            px, py = self._to_px(zx, zy)
            self.screen.blit(surf, (px - int(zr * self.scale),
                                    py - int(zr * self.scale)))
            pygame.draw.circle(self.screen, col, (px, py),
                               int(zr * self.scale), 1)
        for (cx, cy, r) in self.obstacles:
            pygame.draw.circle(self.screen, OBSTACLE, self._to_px(cx, cy),
                               int(r * self.scale))
        if self._p('render_trails'):
            for e, col in ([(a, TRAIL_BOID) for a in self.agents]
                           + ([(self.target, TRAIL_TGT)]
                              if self.target_enabled else [])):
                if len(e.trail) > 1:
                    pygame.draw.lines(
                        self.screen, col, False,
                        [self._to_px(x, y) for (x, y) in e.trail], 1)
        size = int(self._p('agent_radius_px'))
        for a in self.agents:
            self._triangle(a, size, BOID)
        if self.target_enabled:
            tpx = self._to_px(self.target.x, self.target.y)
            self._triangle(self.target, int(size * 1.6), TARGET)
            pygame.draw.circle(self.screen, (70, 45, 45), tpx,
                               int(self._p('d_capture') * self.scale * 0.5), 1)

        # --- HUD (§4.6) ---
        s = self.score
        hud = (f'ep {s.episode}  t={s.t_episode:6.1f}s  '
               f'captures {s.captures}/{max(s.episode - 1, 0)}  '
               f'close {self.n_close}/{int(self._p("capture_k"))}  '
               f'min_d {s.min_pairwise:.2f}')
        self.screen.blit(self.font.render(hud, True, HUD), (8, 6))
        if self._p('target_stamina_enabled') and self.target_enabled:
            pygame.draw.rect(self.screen, (60, 60, 60), (8, 26, 120, 8))
            pygame.draw.rect(self.screen, (240, 200, 60),
                             (8, 26, int(120 * self.stamina), 8))
        if self._p('capture_mode') == 'tag' and self.target_enabled:
            frac = max(self.tag_hp.hp, 0.0) / self.tag_hp.hp_max
            pygame.draw.rect(self.screen, (60, 60, 60), (8, 38, 120, 8))
            pygame.draw.rect(self.screen, TARGET,
                             (8, 38, int(120 * frac), 8))
        if self.state == 'banner':
            txt = self.big_font.render(self.banner_text, True,
                                       (255, 220, 90))
            r = txt.get_rect(center=self.screen.get_rect().center)
            self.screen.blit(txt, r)
        if self.game_mode == 'human' and self.target_enabled:
            tip = self.font.render(
                'HUMAN TARGET: arrows to drive (UP sprint)', True, HUD)
            self.screen.blit(tip, (8, int(self.world * self.scale) - 22))
        if self.paused:
            txt = self.big_font.render('PAUSED', True, (255, 220, 90))
            self.screen.blit(txt, (self.arena_px // 2 - txt.get_width() // 2,
                                   self.arena_px // 2 - 60))
        if self.managed and self.hold:
            self._draw_hold_overlay(pygame)
        if self.panel is not None:
            self.panel.draw(pygame, self.screen, self.arena_px, 0,
                            self.fonts, self._ui_value,
                            self.pstate.is_pending)
        if shot_due:
            os.makedirs(shot_dir, exist_ok=True)
            pygame.image.save(
                self.screen,
                os.path.join(shot_dir, f'shot_{int(self.sim_time):05d}.png'))
            self._next_shot += float(self._p('screenshot_period'))
        if not self.headless:
            pygame.display.flip()


def main(args=None):
    import threading
    import pygame
    rclpy.init(args=args)
    node = PygameSimNode()
    node.setup_display()
    # Never leave the stack's processes behind: atexit covers a normal or
    # exception exit, the handler turns SIGTERM/SIGHUP into a clean loop
    # exit (-> finally -> shutdown). (SIGKILL is covered on the stack's
    # side: stack_supervisor.watch_parent.)
    atexit.register(node.supervisor.shutdown)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: setattr(node, 'running', False))
    node.start_initial_stack()
    # Callbacks run in a background executor thread: rclpy.spin_once in
    # the render loop handles ONE callback per call, which backlogs N
    # cmd_vel streams into ~300ms actuation delay (queue_depth/cmd_rate)
    # and destabilizes every steering loop; per-frame drain loops burn
    # the frame budget instead. The callbacks only do atomic assignments
    # (ent.cmd = tuple), so this stays race-free; pygame stays in the
    # main thread (§1.5).
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    clock = pygame.time.Clock()
    dt = 1.0 / node.fps                       # fixed timestep (§7.4)
    pace = node.fps * (float(node.get_parameter('time_scale').value)
                       if node.headless else 1.0)
    t_wall0, t_rep, frames = _time.monotonic(), 0.0, 0
    try:
        while rclpy.ok() and node.running:
            clock.tick(pace)              # real time; headless: bounded FF
            node.handle_pygame_events()
            node.tick_stack()
            if not node.paused:
                node.sim_time += dt
                # /clock keeps running while the world is held (Nav2 needs
                # it to activate), physics and the score do not.
                if not node.hold:
                    node.step_physics(dt)
                    node.check_capture_and_score(dt)
            # Keep publishing while paused: /clock stops advancing, so the
            # controllers' sim-time timers freeze with the world instead of
            # spinning on a stale pose and tripping their pose_timeout.
            node.publish_states()
            node.render()
            frames += 1
            if node.sim_time - t_rep >= 10.0:        # health report
                wall = _time.monotonic() - t_wall0
                node.get_logger().info(
                    f'sim_t={node.sim_time:.0f}s wall_t={wall:.0f}s '
                    f'speed={node.sim_time / max(wall, 1e-9):.2f}x '
                    f'fps={frames / max(wall, 1e-9):.0f}')
                t_rep = node.sim_time
    except KeyboardInterrupt:
        pass
    finally:
        node.supervisor.shutdown()
        executor.shutdown(timeout_sec=1.0)
        pygame.quit()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
