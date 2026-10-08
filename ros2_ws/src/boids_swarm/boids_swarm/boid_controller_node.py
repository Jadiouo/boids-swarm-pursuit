"""boid_controller_node — per-agent flocking + pursuit (SDD v3 §3–4).

Unchanged pub/sub contract from v2: poses in, one Twist out per cycle.
Sensing (pose callbacks -> cache) is decoupled from control (fixed-rate
timer); pursuit tactics are strategy objects picked by parameter (§7.2).
"""

import math
import random
import re
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from turtlesim.msg import Pose
from boids_swarm_msgs.msg import EpisodeState, TargetSighting
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from .geometry import to_twist, wrap_angle
from .behaviors import flocking
from .behaviors.pursuit import STRATEGIES, PursuitContext, terminal_commit
from . import perception
from .tracking import (Tracker, CirclingDetector,
                       target_state_from_track)
from .sighting_protocol import SightingRecord, validate_sighting
from .sighting_runtime import (ObservationInbox, SightingCounters,
                               shared_sighting_qos_spec)

TELEPORT_JUMP = 3.0        # target jump > this ⇒ episode reset ⇒ re-engage


class CachedPose:
    __slots__ = ('x', 'y', 'theta', 'v', 'w', 'stamp')

    def __init__(self, x, y, theta, v, w, stamp):
        self.x, self.y, self.theta, self.v, self.w = x, y, theta, v, w
        self.stamp = stamp


def target_pose_from_track(track, observation_stamp):
    """Expose filtered target motion while retaining source-time freshness."""
    x, y, heading, speed, stamp = target_state_from_track(
        track, observation_stamp)
    return CachedPose(x, y, heading, speed, 0.0, stamp)


class BoidController(Node):

    def __init__(self):
        super().__init__('boid_controller')
        # --- flocking params (v2 §5) ---
        self.declare_parameter('num_agents', 12)
        self.declare_parameter('sensing_radius', 4.0)
        self.declare_parameter('safe_distance', 1.2)
        self.declare_parameter('w_separation', 1.5)
        self.declare_parameter('w_alignment', 0.5)
        self.declare_parameter('w_cohesion', 0.5)
        self.declare_parameter('w_boundary', 2.0)
        self.declare_parameter('w_obstacle', 2.0)
        self.declare_parameter('w_wander', 0.5)
        self.declare_parameter('k_linear', 1.2)
        self.declare_parameter('k_angular', 5.0)
        self.declare_parameter('v_min', 0.15)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 6.0)
        self.declare_parameter('bounds_min', [0.0, 0.0])
        self.declare_parameter('bounds_max', [20.0, 20.0])
        self.declare_parameter('boundary_margin', 1.2)
        self.declare_parameter('obstacle_margin', 1.0)
        self.declare_parameter('obstacles', [0.0])      # flat [x,y,r,...]
        self.declare_parameter('wander_jitter', 0.3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pose_timeout', 1.0)
        # --- pursuit params (v3 §5) ---
        self.declare_parameter('target_enabled', True)
        self.declare_parameter('w_pursuit', 2.0)
        self.declare_parameter('pursuit_strategy', 'intercept')
        self.declare_parameter('lead_time', 1.0)
        self.declare_parameter('agent_max_speed', 2.0)
        self.declare_parameter('ring_radius_start', 3.0)
        self.declare_parameter('ring_radius_min', 0.8)
        self.declare_parameter('ring_shrink_rate', 0.2)
        self.declare_parameter('herd_corner_dist', 4.5)
        self.declare_parameter('commit_distance', 2.5)   # pounce range (M15)
        self.declare_parameter('debug_period', 0.0)     # sec; 0 = off
        # stability aids (latency compensation / anti-jitter)
        self.declare_parameter('v_deadband', 0.12)  # |V| below: coast
        self.declare_parameter('v_filter_alpha', 0.35)  # EMA on V
        # perception source (v4 A.7): 'perfect' = v3 /swarm/poses + /target/
        # pose ground truth; 'sensor' = own odometry + /agentK/detections.
        self.declare_parameter('perception_mode', 'perfect')
        # --- tracking + search (v4 A.5–A.6, sensor mode) ---
        self.declare_parameter('track_alpha', 0.5)
        self.declare_parameter('track_beta', 0.12)
        self.declare_parameter('track_gate', 1.8)
        self.declare_parameter('track_max_age', 0.6)
        self.declare_parameter('w_search', 1.0)
        self.declare_parameter('search_enabled', True)
        self.declare_parameter('world_size', 20.0)
        self.declare_parameter('sharing_mode', 'legacy')
        self.declare_parameter('radio_range', 8.0)
        self.declare_parameter('shared_sighting_qos_depth', 10)
        self.declare_parameter('relay_log_dir', '')
        self.declare_parameter('radio_pose_time_tolerance', 0.15)
        self.declare_parameter('sighting_timeout', 0.6)
        self.declare_parameter('future_stamp_tolerance', 0.02)
        self.add_on_set_parameters_callback(self._on_params)
        self._dbg_next = 0.0
        self._v_filt = (0.0, 0.0)
        self._last_w = 0.0

        ns = self.get_namespace().strip('/')            # e.g. 'agent3'
        m = re.fullmatch(r'agent(\d+)', ns)
        if not m:
            raise RuntimeError(
                'boid_controller must run in an /agentK namespace')
        self.ns, self.index = ns, int(m.group(1))
        self.n_agents = int(self.get_parameter('num_agents').value)
        self._warned_strategy = None

        self.cache: dict[str, CachedPose] = {}
        self.target_pose: CachedPose | None = None
        self.wander_angle = 0.0
        self._rng = random.Random(self.index)           # §7.4 determinism
        self._t_first_seen: float | None = None         # engagement clock
        self._circling = CirclingDetector()             # v4 M11 trigger

        self.target_enabled = bool(self.get_parameter('target_enabled').value)
        self.perception_mode = self.get_parameter('perception_mode').value
        self.sharing_mode = self.get_parameter('sharing_mode').value
        self._episode_id = None
        self._inbox = ObservationInbox(
            self_id=self.ns, expected_episode=None,
            sighting_timeout=self._p('sighting_timeout'),
            future_tolerance=self._p('future_stamp_tolerance'),
            max_valid_for=2.0)
        self._status_seq = 0
        self._last_track_mode = 'search'
        self._target_valid_until = float('-inf')
        self._last_applied_candidate = None
        self._last_control_clock = None
        self.track_status_pub = None
        self.shared_sighting_qos_depth = shared_sighting_qos_spec(
            self.get_parameter('shared_sighting_qos_depth').value)['depth']
        self.shared_sighting_qos = QoSProfile(
            depth=self.shared_sighting_qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._counters = SightingCounters()
        # Optional reliable file record of the keys this controller really
        # published to the shared topic (the per-receiver loss denominator
        # must only contain keys that reached the wire).
        self._published_log = None
        log_dir = self.get_parameter('relay_log_dir').value
        if log_dir:
            import os
            os.makedirs(log_dir, exist_ok=True)
            self._published_log = open(
                os.path.join(log_dir, f'published_{self.ns}.jsonl'), 'a',
                buffering=1)
        self.tracker = None
        self.target_tracker = None
        if self.perception_mode == 'sensor':
            # v4 Part A: own odometry + own synthesized detection stream.
            # No global neighbor/target ground truth.
            self.tracker = Tracker(
                alpha=self._p('track_alpha'), beta=self._p('track_beta'),
                gate=self._p('track_gate'), max_age=self._p('track_max_age'))
            if self.sharing_mode == 'ros':
                self.target_tracker = Tracker(
                    alpha=self._p('track_alpha'), beta=self._p('track_beta'),
                    gate=self._p('track_gate'),
                    max_age=self._p('track_max_age'))
            self.create_subscription(Pose, f'/{self.ns}/pose',
                                     self._on_own_pose, 10)
            self.create_subscription(Float32MultiArray,
                                     f'/{self.ns}/detections',
                                     self._on_detections, 10)
            epoch_qos = QoSProfile(depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(EpisodeState,
                '/simulation/episode_state', self._on_episode_state,
                epoch_qos)
            self.create_subscription(TargetSighting,
                f'/{self.ns}/local_target_sighting', self._on_local_sighting, 10)
            self.create_subscription(TargetSighting,
                '/swarm/target_sightings', self._on_shared_sighting,
                self.shared_sighting_qos)
            self.shared_sighting_pub = self.create_publisher(
                TargetSighting, '/swarm/target_sightings',
                self.shared_sighting_qos)
            self.relay_event_pub = self.create_publisher(
                String, '/swarm/relay_events', 50)
            self.track_status_pub = self.create_publisher(
                String, f'/{self.ns}/target_track_status', 20)
        else:
            # v3 baseline: ONE aggregate subscription for all peer poses
            # (v2 §10.2) keeps total subs O(N) instead of O(N^2), plus the
            # global target pose.
            self.create_subscription(Float32MultiArray, '/swarm/poses',
                                     self._on_swarm_poses, 10)
            if self.get_parameter('target_enabled').value:
                self.create_subscription(Pose, '/target/pose',
                                         self._on_target_pose, 10)
        # live arena bounds (v4 C.4 shrinking arena): override the static
        # bounds params so boundary avoidance tracks the moving wall.
        self._live_bounds = None
        self.create_subscription(Float32MultiArray, '/arena/bounds',
                                 self._on_bounds, 10)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.control_cycle)
        self.get_logger().info(
            f'boid {self.index}/{self.n_agents} up, '
            f"strategy={self.get_parameter('pursuit_strategy').value}")

    def _now(self) -> float:
        """Node-clock seconds — follows /clock when use_sim_time is set,
        so freshness/engagement stay correct under headless fast-forward."""
        return self.get_clock().now().nanoseconds * 1e-9

    # --- sensing: cheap callbacks, cache only (§7.2) ----------------------
    def _on_swarm_poses(self, msg: Float32MultiArray):
        d = msg.data                       # [N, x0,y0,th0,v0,w0, x1,...]
        now = self._now()
        n = int(d[0])
        for i in range(n):
            b = 1 + 5 * i
            self.cache[f'agent{i}'] = CachedPose(
                d[b], d[b + 1], d[b + 2], d[b + 3], d[b + 4], now)

    def _on_target_pose(self, msg: Pose):
        self._note_target(msg.x, msg.y, msg.theta, msg.linear_velocity,
                          msg.angular_velocity)

    def _note_target(self, x, y, theta, v, w):
        prev = self.target_pose
        now = self._now()
        if prev is not None and \
                math.hypot(x - prev.x, y - prev.y) > TELEPORT_JUMP:
            self._t_first_seen = now          # respawn ⇒ re-engage ring
        if self._t_first_seen is None:
            self._t_first_seen = now
        self.target_pose = CachedPose(x, y, theta, v, w, now)

    @staticmethod
    def _sighting_record(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return SightingRecord(
            episode_id=msg.episode_id, sequence=msg.sequence,
            sender_id=msg.sender_id, target_id=msg.target_id,
            frame_id=msg.header.frame_id, stamp=stamp,
            sender_xy=(msg.sender_position.x, msg.sender_position.y),
            target_xy=(msg.target_position.x, msg.target_position.y),
            covariance_xy=tuple(msg.covariance_xy),
            confidence=msg.confidence, valid_for_sec=msg.valid_for_sec)

    def _on_episode_state(self, msg):
        changed = self._inbox.set_episode(int(msg.episode_id))
        if changed != 'episode_changed':
            return
        self._episode_id = int(msg.episode_id)
        self.cache.clear()
        self.target_pose = None
        self._t_first_seen = None
        self._circling = CirclingDetector()
        if self.tracker is not None:
            self.tracker.clear()
        if self.target_tracker is not None:
            self.target_tracker.clear()
        self._publish_track_status(False, 'search', 'episode_reset', None,
                                   event='transition')

    def _on_local_sighting(self, msg):
        if self.sharing_mode != 'ros':
            return
        record = self._sighting_record(msg)
        if self._episode_id is None:
            return
        reason = self._inbox.offer(record, source='direct', now=self._now())
        if reason is None:
            # Publish the unchanged simulator observation once; relayed
            # observations never enter this path.
            self.shared_sighting_pub.publish(msg)
            self._counters.on_published()
            if self._published_log is not None:
                self._published_log.write(json.dumps(
                    [int(record.episode_id), record.sender_id,
                     int(record.sequence), record.stamp]) + '\n')

    def _on_shared_sighting(self, msg):
        if self.sharing_mode != 'ros':
            return
        record = self._sighting_record(msg)
        now = self._now()
        self._count_received(record, now)
        reason = None
        me = self.cache.get(self.ns)
        if record.sender_id == self.ns:
            reason = 'self_originated'
        elif self._episode_id is None:
            reason = 'awaiting_episode'
        else:
            reason = validate_sighting(
                record, expected_episode=self._episode_id, now=now,
                sighting_timeout=self._p('sighting_timeout'),
                future_tolerance=self._p('future_stamp_tolerance'),
                max_valid_for=2.0)
        if reason is None and (me is None or now - me.stamp >
                               self._p('pose_timeout')):
            reason = 'receiver_pose_unavailable'
        elif reason is None and abs(me.stamp - record.stamp) > self._p(
                'radio_pose_time_tolerance'):
            reason = 'receiver_pose_time_mismatch'
        elif reason is None:
            distance = math.hypot(me.x - record.sender_xy[0],
                                  me.y - record.sender_xy[1])
            if distance > self._p('radio_range'):
                reason = 'out_of_range'
            else:
                reason = self._inbox.offer(record, source='relay', now=now)
        self._publish_relay_event(msg, now, reason)

    def _count_received(self, record, now):
        """R-03: classify every callback-delivered shared sighting, before
        and independently of the validity/freshness gates."""
        is_self = record.sender_id == self.ns
        me = self.cache.get(self.ns)
        distance = None
        if not is_self and me is not None and \
                now - me.stamp <= self._p('pose_timeout'):
            distance = math.hypot(me.x - record.sender_xy[0],
                                  me.y - record.sender_xy[1])
        self._counters.on_received(is_self, distance,
                                   self._p('radio_range'))

    def _publish_relay_event(self, msg, now, reason):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        me = self.cache.get(self.ns)
        distance = None if me is None else math.hypot(
            me.x-msg.sender_position.x, me.y-msg.sender_position.y)
        payload = {'receiver_id': self.ns, 'sender_id': msg.sender_id,
                   'episode_id': msg.episode_id, 'sequence': msg.sequence,
                   'observation_stamp': stamp, 'receive_stamp': now,
                   'age': now-stamp, 'radio_distance': distance,
                   'opportunity': reason in ('out_of_range', None),
                   'accepted': reason is None,
                   'reason': 'accepted' if reason is None else reason}
        self.relay_event_pub.publish(String(data=json.dumps(payload)))

    def _publish_track_status(self, valid, mode, reason, candidate,
                              event='cycle'):
        self._status_seq += 1
        payload = {'episode_id': self._episode_id,
                   'control_cycle_sequence': self._status_seq,
                   'stamp': self._now(), 'belief_valid': bool(valid),
                   'mode': mode, 'reason': reason, 'event': event,
                   'sharing_mode': self.sharing_mode,
                   'shared_sighting_qos_depth': self.shared_sighting_qos_depth,
                   **self._counters.snapshot(),
                   'source': ('unknown' if mode in ('oracle_tracking',
                       'legacy_tracking') else mode),
                   'own_pose_fresh': bool(self.cache.get(self.ns) is not None
                       and self._now() - self.cache[self.ns].stamp <=
                       self._p('pose_timeout'))}
        if candidate is not None:
            r = candidate.record
            payload.update(last_sender_id=r.sender_id,
                           last_sequence=r.sequence,
                           observation_stamp=r.stamp)
        if self.track_status_pub is not None:
            self.track_status_pub.publish(String(data=json.dumps(payload)))

    # --- sensor mode (v4 A.5): own odometry + relative detections --------
    def _on_bounds(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) >= 4:
            self._live_bounds = ([d[0], d[1]], [d[2], d[3]])

    def _bounds(self):
        """Live bounds if the arena is shrinking, else the static params."""
        if self._live_bounds is not None:
            return self._live_bounds
        return self._p('bounds_min'), self._p('bounds_max')

    def _on_own_pose(self, msg: Pose):
        self.cache[self.ns] = CachedPose(
            msg.x, msg.y, msg.theta, msg.linear_velocity,
            msg.angular_velocity, self._now())

    def _on_detections(self, msg: Float32MultiArray):
        me = self.cache.get(self.ns)
        if me is None:
            return                            # need own pose to place blips
        now = self._now()
        neighbors, target = perception.flat_to_world(
            list(msg.data), (me.x, me.y, me.theta))
        # Feed world-frame blips through the track filter (v4 A.5): smooths
        # noise, bridges dropouts, derives heading/speed from velocity.
        dets = [(x, y, cid, False) for (x, y, _th, cid) in neighbors]
        if target is not None and self.sharing_mode != 'ros':
            dets.append((target[0], target[1], target[4], True))
        ntracks, ttrack = self.tracker.step(dets, now)

        # rebuild the neighbor cache from confident tracks only
        self.cache = {k: v for k, v in self.cache.items() if k == self.ns}
        for tr in ntracks:
            self.cache[f'trk{tr.tid}'] = CachedPose(
                tr.x, tr.y, tr.heading, tr.speed, 0.0, now)
        if ttrack is not None and self.sharing_mode != 'ros':
            # heading/speed from the track velocity, not the raw blip
            self._note_target(ttrack.x, ttrack.y, ttrack.heading,
                              ttrack.speed, 0.0)

    def _on_params(self, params):
        for p in params:
            if p.name == 'control_rate_hz':
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='rate must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / p.value,
                                               self.control_cycle)
        return SetParametersResult(successful=True)

    def _p(self, name):
        return self.get_parameter(name).value

    def _strategy(self):
        """Resolve `pursuit_strategy`, warning ONCE per bad name.

        Silently falling back would make a typo in a benchmark sweep look
        like a real result for the strategy that was never run.
        """
        name = self._p('pursuit_strategy')
        fn = STRATEGIES.get(name)
        if fn is None:
            if self._warned_strategy != name:
                self._warned_strategy = name
                self.get_logger().warn(
                    f"unknown pursuit_strategy '{name}' — falling back to "
                    f"'intercept'. Valid: {'|'.join(STRATEGIES)}")
            return STRATEGIES['intercept']
        return fn

    # --- control cycle (v2 §7 + pursuit term) ------------------------------
    def control_cycle(self):
        now = self._now()
        if self._last_control_clock is not None and \
                now + 1e-9 < self._last_control_clock:
            self.cache.clear()
            self.target_pose = None
            self._target_valid_until = float('-inf')
            self._t_first_seen = None
            self._inbox.on_clock_rollback()
            if self.tracker is not None:
                self.tracker.clear()
            if self.target_tracker is not None:
                self.target_tracker.clear()
            self._publish_track_status(False, 'search', 'clock_rollback', None,
                                       event='transition')
        self._last_control_clock = now
        me = self.cache.get(self.ns)
        if me is None:
            return
        candidate = None
        if self.sharing_mode == 'ros' and self._episode_id is not None:
            candidate = self._inbox.take_for_cycle(now=now)
            if candidate is not None:
                record = candidate.record
                track = self.target_tracker.observe_target(
                    *record.target_xy, record.stamp)
                self.target_pose = target_pose_from_track(track, record.stamp)
                self._target_valid_until = record.stamp + min(
                    record.valid_for_sec, self._p('sighting_timeout'))
                self._t_first_seen = (self._t_first_seen or record.stamp)
                self._inbox.mark_applied(candidate)
                self._last_applied_candidate = candidate
        if self.sharing_mode == 'ros' and self.target_pose is not None \
                and now > self._target_valid_until:
            self.target_pose = None
            self.target_tracker.forget_target()
        timeout = self._p('pose_timeout')
        r = self._p('sensing_radius')
        me_xy = (me.x, me.y)

        neighbors = []
        for name, p in self.cache.items():
            if name == self.ns or now - p.stamp > timeout:
                continue
            if math.hypot(p.x - me.x, p.y - me.y) < r:
                neighbors.append((p.x, p.y, p.theta))

        tgt = self.target_pose
        target_fresh = tgt is not None and now - tgt.stamp <= timeout
        if self.sharing_mode == 'ros':
            target_fresh = target_fresh and now <= self._target_valid_until
        # search: sensor mode, target expected but currently unseen by us
        searching = (self.perception_mode == 'sensor' and self.target_enabled
                     and not target_fresh and self._p('search_enabled'))

        vx = vy = 0.0
        # Separation always applies when we have neighbors (collision safety),
        # so it is on during flocking, pursuit AND search.
        if neighbors:
            sx, sy = flocking.separation(me_xy, neighbors,
                                         self._p('safe_distance'))
            vx += self._p('w_separation') * sx
            vy += self._p('w_separation') * sy

        if target_fresh and neighbors:
            ax, ay = flocking.alignment(neighbors)
            cx, cy = flocking.cohesion(me_xy, neighbors)
            vx += self._p('w_alignment') * ax + self._p('w_cohesion') * cx
            vy += self._p('w_alignment') * ay + self._p('w_cohesion') * cy
        elif searching:
            # coverage sweep until someone reacquires (v4 A.6)
            sxv, syv = flocking.search_vec(
                me_xy, self.index, self.n_agents,
                self._p('world_size'), now)
            vx += self._p('w_search') * sxv
            vy += self._p('w_search') * syv
        elif not target_fresh and neighbors:
            # no target expected (e.g. pure-flocking run): full flock
            ax, ay = flocking.alignment(neighbors)
            cx, cy = flocking.cohesion(me_xy, neighbors)
            vx += self._p('w_alignment') * ax + self._p('w_cohesion') * cx
            vy += self._p('w_alignment') * ay + self._p('w_cohesion') * cy
        elif not target_fresh and not searching:
            # lone agent, nothing to do: wander
            jitter = self._p('wander_jitter')
            self.wander_angle = wrap_angle(
                self.wander_angle + self._rng.uniform(-jitter, jitter))
            wx, wy = flocking.wander_vec(me.theta, self.wander_angle)
            vx += self._p('w_wander') * wx
            vy += self._p('w_wander') * wy

        if target_fresh:
            strategy = self._strategy()
            bmin, bmax = self._bounds()
            center = (0.5 * (bmin[0] + bmax[0]), 0.5 * (bmin[1] + bmax[1]))
            half_extent = 0.5 * min(bmax[0] - bmin[0], bmax[1] - bmin[1])
            circling, circ_dir, orbit_r = self._circling.update(
                now, (tgt.x, tgt.y), center, half_extent)
            ctx = PursuitContext(
                self_xy=me_xy, self_theta=me.theta,
                index=self.index, n_agents=self.n_agents,
                target_xy=(tgt.x, tgt.y), target_theta=tgt.theta,
                target_speed=tgt.v,
                t_engaged=now - (self._t_first_seen or now),
                center=center, circling=circling, circ_dir=circ_dir,
                orbit_radius=orbit_r)
            params = {
                'lead_time': self._p('lead_time'),
                'agent_max_speed': self._p('agent_max_speed'),
                'ring_radius_start': self._p('ring_radius_start'),
                'ring_radius_min': self._p('ring_radius_min'),
                'ring_shrink_rate': self._p('ring_shrink_rate'),
                'herd_corner_dist': self._p('herd_corner_dist'),
                'commit_distance': self._p('commit_distance'),
                'bounds_min': bmin,
                'bounds_max': bmax,
            }
            px, py = terminal_commit(ctx, params, strategy(ctx, params))
            vx += self._p('w_pursuit') * px
            vy += self._p('w_pursuit') * py

        if self.perception_mode == 'sensor':
            if self.sharing_mode == 'ros':
                track_mode = (
                    'direct' if self._last_applied_candidate is not None and
                    self._last_applied_candidate.source == 'direct' else
                    'relay') if target_fresh else 'search'
                candidate_for_status = (self._last_applied_candidate
                                         if target_fresh else None)
            elif target_fresh and self.sharing_mode == 'oracle':
                track_mode, candidate_for_status = 'oracle_tracking', None
            elif target_fresh and self.sharing_mode == 'legacy':
                track_mode, candidate_for_status = 'legacy_tracking', None
            else:
                track_mode = 'direct' if target_fresh else 'search'
                candidate_for_status = None
            self._publish_track_status(
                target_fresh, track_mode,
                'tracking' if target_fresh else 'no_fresh_sighting',
                candidate_for_status)

        b_lo, b_hi = self._bounds()
        bx, by = flocking.boundary(me_xy, b_lo, b_hi,
                                   self._p('boundary_margin'))
        vx += self._p('w_boundary') * bx
        vy += self._p('w_boundary') * by

        obstacles = [tuple(self._p('obstacles')[i:i + 3])
                     for i in range(0, len(self._p('obstacles')) - 2, 3)]
        if obstacles:
            # pass the intended velocity so avoidance slides tangentially
            # around plugs instead of stalling in front of them
            ox, oy = flocking.obstacle_avoid(me_xy, obstacles,
                                             self._p('obstacle_margin'),
                                             desired=(vx, vy))
            vx += self._p('w_obstacle') * ox
            vy += self._p('w_obstacle') * oy

        # --- stability aids ------------------------------------------------
        # EMA-smooth the desired vector: async sensing makes the raw sum
        # jitter cycle-to-cycle, which chatters the steering P controller.
        alpha = self._p('v_filter_alpha')
        self._v_filt = (alpha * vx + (1 - alpha) * self._v_filt[0],
                        alpha * vy + (1 - alpha) * self._v_filt[1])
        fx, fy = self._v_filt
        # Deadband: near a flock equilibrium |V|~0 and atan2 is pure
        # noise. Stop translating, but keep rotating toward the local
        # mean heading — a frozen random heading would lock the whole
        # clump out of alignment consensus (and out of quick reactions).
        if math.hypot(fx, fy) < self._p('v_deadband'):
            self._last_w = 0.0
            twist = Twist()
            if neighbors:
                ax, ay = flocking.alignment(neighbors)
                if ax or ay:
                    age = now - me.stamp
                    theta_est = wrap_angle(me.theta + me.w * min(age, 0.2))
                    e = wrap_angle(math.atan2(ay, ax) - theta_est)
                    twist.angular.z = max(-self._p('w_max'),
                                          min(self._p('w_max'),
                                              0.5 * self._p('k_angular') * e))
            self.cmd_pub.publish(twist)
            return
        # Latency compensation: our cached heading is up to one pose
        # period old; extrapolate by the published turn rate.
        age = now - me.stamp
        theta_est = wrap_angle(me.theta + me.w * min(age, 0.2))

        v_lin, w_z = to_twist((fx, fy), theta_est,
                              self._p('k_linear'), self._p('k_angular'),
                              self._p('v_min'), self._p('v_max'),
                              self._p('w_max'), last_w=self._last_w)
        self._last_w = w_z

        dbg = self._p('debug_period')
        if dbg > 0.0 and now >= self._dbg_next:
            self._dbg_next = now + dbg
            self.get_logger().info(
                f'cache={len(self.cache)} nb={len(neighbors)} '
                f'me=({me.x:.1f},{me.y:.1f},{me.theta:.2f}) '
                f'V=({vx:.2f},{vy:.2f}) cmd=({v_lin:.2f},{w_z:.2f}) '
                f'tgt_fresh={target_fresh}')
        twist = Twist()
        twist.linear.x = v_lin
        twist.angular.z = w_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BoidController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
