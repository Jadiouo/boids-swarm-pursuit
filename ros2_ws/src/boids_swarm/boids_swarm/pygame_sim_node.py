"""pygame_sim_node — THE WORLD (SDD v3 §1).

Sole owner of world state: integrates non-holonomic/holonomic physics for
all agents + target, detects capture, scores, renders, publishes poses.
Controllers talk to it only via ROS topics (pose out, cmd_vel in).

Single-threaded pygame + rclpy loop per §1.5. Physics uses a FIXED
timestep (1/fps) so seeded runs are comparable (§7.4 determinism);
`headless:=true` skips rendering and runs unthrottled (fast-forward).
"""

import math
import os
import random

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
from turtlesim.msg import Pose

from .geometry import clamp, wrap_angle
from .game import (Scoreboard, TagHealth, capture_escape_blocked,
                   capture_hull)
from . import perception
from . import comms
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
        env_type = self.get_parameter('env_type').value
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
        self.keys = None                          # set by run loop (human mode)
        self._next_shot = float(self.get_parameter('screenshot_period').value)

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
        self.det_pubs = [
            self.create_publisher(Float32MultiArray, f'/agent{i}/detections',
                                  10)
            for i in range(self.n)]
        self.percept_rng = random.Random(
            int(self.get_parameter('seed').value) + 104729)
        self.frame = 0
        self.pose_div = max(1, round(
            self.fps / float(self.get_parameter('pose_rate_hz').value)))
        self.pose_pubs = [
            self.create_publisher(Pose, f'/agent{i}/pose', 10)
            for i in range(self.n)]
        # depth=1: only the LATEST command matters — a deeper queue holds
        # stale commands and turns into actuation delay (see main loop).
        for i in range(self.n):
            self.create_subscription(
                Twist, f'/agent{i}/cmd_vel',
                lambda m, k=i: self._on_cmd(m, self.agents[k]), 1)
        if self.target_enabled:
            self.target_pose_pub = self.create_publisher(
                Pose, '/target/pose', 10)
            self.create_subscription(
                Twist, '/target/cmd_vel',
                lambda m: self._on_cmd(m, self.target), 1)

        self._spawn_all()
        self.score.start_episode()
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
                if self._p('auto_reset'):
                    self._spawn_all()
                    self.score.start_episode()
                    self.state = 'running'
                em = int(self._p('episodes_max'))
                if em and self.score.episode > em:
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
        line = (f'SUMMARY episodes={s.episode - 1} captures={s.captures} '
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

    def _publish_detections(self):
        """Synthesize each agent's own detection stream (v4 A.2), then
        relay target sightings over the comm mesh (v4 A.6)."""
        cfg = self._sensor_cfg()
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

        # pass 2: mesh relay — informed agents that did NOT see the target
        # directly get a relayed sighting in their own frame.
        if (self.target_enabled and self._p('comms_enabled')
                and any(seers)):
            positions = [(a.x, a.y) for a in self.agents]
            informed = comms.propagate_sightings(
                positions, self._p('comm_range'), seers)
            jitter = self._p('comm_jitter')
            tgt = (self.target.x, self.target.y, self.target.theta,
                   self.target.v)
            for i, a in enumerate(self.agents):
                if informed[i] and not seers[i]:
                    per_agent[i].append(perception.relayed_detection(
                        (a.x, a.y, a.theta), tgt, self.n, jitter,
                        self.percept_rng))

        for i in range(self.n):
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
        self.screen = pygame.display.set_mode((px, px))
        pygame.display.set_caption('boids_swarm — cooperative pursuit')
        self.font = pygame.font.SysFont('monospace', 15)
        self.big_font = pygame.font.SysFont('monospace', 34, bold=True)
        self.scale = px / self.world

    def _to_px(self, x, y):
        return int(x * self.scale), int((self.world - y) * self.scale)

    def handle_pygame_events(self):
        import pygame
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (
                    ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                self.running = False
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
    import time as _time
    import pygame
    rclpy.init(args=args)
    node = PygameSimNode()
    node.setup_display()
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
            node.sim_time += dt
            node.step_physics(dt)
            node.check_capture_and_score(dt)
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
        executor.shutdown(timeout_sec=1.0)
        pygame.quit()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
