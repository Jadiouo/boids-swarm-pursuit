"""target_controller_node — the evader (SDD v3 §4.2–4.3, §6.3).

Pluggable brain behind the `evader` parameter: `reactive` (default),
`adaptive` (v4 M14 utility selector), `smart` (geodesic escape planner,
no Nav2) or `nav2` (M7 stretch). Publishes
/target/cmd_vel; the sim enforces the speed cap and the turn-rate limit
that keeps the game winnable.
"""

import json
import math
import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from turtlesim.msg import Pose

from .geometry import to_twist
from .behaviors.evasion import AdaptiveEvader, ReactiveEvader
from .behaviors.nav2_evader import Nav2Evader
from .behaviors.smart_evader import SmartConfig, SmartEvader


class TargetController(Node):

    def __init__(self):
        super().__init__('target_controller')
        self.declare_parameter('num_agents', 12)
        self.declare_parameter('evader', 'reactive')  # reactive|adaptive|smart|nav2
        self.declare_parameter('agent_max_speed', 2.0)
        self.declare_parameter('target_speed_multiplier', 2.0)
        self.declare_parameter('target_omega_max', 2.5)
        self.declare_parameter('bounds_min', [0.0, 0.0])
        self.declare_parameter('bounds_max', [20.0, 20.0])
        self.declare_parameter('obstacles', [0.0])
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('pose_timeout', 1.0)
        # Sprint management (§4.2 stamina): sprint only when pursuers are
        # close, cruise at swarm speed otherwise so stamina regenerates.
        self.declare_parameter('panic_distance', 6.0)
        self.declare_parameter('seed', 7)
        # Nav2 evader (M7). Read live by Nav2Evader via node parameters.
        self.declare_parameter('nav2_goal_period', 0.75)    # s between goal picks
        self.declare_parameter('nav2_replan_period', 2.0)   # s, re-plan same goal
        self.declare_parameter('nav2_reactive_radius', 4.0)  # blend starts here
        self.declare_parameter('nav2_reactive_min', 1.5)     # pure reactive inside
        self.declare_parameter('nav2_cmd_timeout', 0.3)     # stale nav2 cmd -> reactive
        self.declare_parameter('nav2_fallback_hold', 1.0)   # s reactive after failure
        self.declare_parameter('nav2_plan_timeout', 2.0)
        self.declare_parameter('nav2_blacklist_ttl', 6.0)   # s a failed goal stays banned
        self.declare_parameter('nav2_blacklist_radius', 1.0)
        # the rest of Nav2Evader.DEFAULTS (selector, dom_margin, smooth_start,
        # goal_clearance, planner_radius, map_resolution, ...): undeclared
        # launch params would be silently ignored
        Nav2Evader.declare_params(self)
        # Smart evader: strategy/weights overridable (`ros2 param set`, then
        # re-select evader to rebuild). Defaults live in SmartConfig.
        for _k in self.SMART_TUNABLES:
            self.declare_parameter('smart_' + _k, getattr(SmartConfig(), _k))
        # mirrors of the sim's stamina model (params.yaml sets both sides)
        self.declare_parameter('target_stamina_enabled', False)
        self.declare_parameter('target_stamina_drain', 0.25)
        self.declare_parameter('target_stamina_regen', 0.35)
        # same value the sim collides with and Nav2's costmaps use
        self.declare_parameter('target_body_radius', 0.7)

        flat = [float(v) for v in self.get_parameter('obstacles').value]
        obstacles = [tuple(flat[i:i + 3])
                     for i in range(0, len(flat) - 2, 3)]

        self._obstacles = obstacles
        self.kind = None
        self.brain = None
        self._last_mode = None
        self._set_brain(self.get_parameter('evader').value)
        # Swapping the evader at runtime (the GUI's brain cycler, M15) just
        # means rebuilding the strategy object — it holds no ROS state.
        self.add_on_set_parameters_callback(self._on_params)

        self.pose = None
        self.pursuers: dict[int, tuple] = {}
        self.create_subscription(Pose, '/target/pose', self._on_own_pose, 10)
        # one aggregate subscription for all pursuer poses (v2 §10.2)
        self.create_subscription(Float32MultiArray, '/swarm/poses',
                                 self._on_swarm_poses, 10)
        # live arena bounds (v4 C.4): keep the evader's wall model current
        self.create_subscription(Float32MultiArray, '/arena/bounds',
                                 self._on_bounds, 10)
        self.cmd_pub = self.create_publisher(Twist, '/target/cmd_vel', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.create_timer(1.0 / rate, self.control_cycle)
        self._last_w = 0.0
        self.get_logger().info(
            f'target up: evader={self.kind}, v_max={self._v_max():.2f}')

    def _v_max(self):
        """Read the caps fresh each cycle: the sim enforces the real limits
        and the GUI can retune them live, so caching at startup would leave
        the evader steering to a speed the world no longer allows."""
        return (self.get_parameter('agent_max_speed').value
                * self.get_parameter('target_speed_multiplier').value)

    BRAINS = {'reactive': ReactiveEvader, 'adaptive': AdaptiveEvader,
              'smart': SmartEvader, 'nav2': Nav2Evader}

    # smart_<name> ROS parameters (defaults come from SmartConfig); the
    # speeds / omega / panic / stamina values are derived from the sim's own
    # parameters below so they cannot drift from the world.
    SMART_TUNABLES = ('selector', 'dom_margin', 'w_dom', 'decision_hz',
                      'candidates', 'w_lead', 'w_open',
                      'w_away', 'w_dead', 'hysteresis', 'w_progress',
                      'w_danger', 'w_clear', 'w_enclose', 'horizon',
                      'rollouts')

    def _smart_config(self):
        params = {f'smart_{k}': self.get_parameter(f'smart_{k}').value
                  for k in self.SMART_TUNABLES}
        params['smart_control_rate_hz'] = self.get_parameter(
            'control_rate_hz').value
        params['smart_target_speed'] = self._v_max()
        params['smart_pursuer_speed'] = self.get_parameter(
            'agent_max_speed').value
        params['smart_cruise_speed'] = params['smart_pursuer_speed']
        params['smart_omega_max'] = self.get_parameter(
            'target_omega_max').value
        params['smart_panic_distance'] = self.get_parameter(
            'panic_distance').value
        params['smart_use_stamina'] = int(bool(self.get_parameter(
            'target_stamina_enabled').value))
        params['smart_stamina_drain'] = self.get_parameter(
            'target_stamina_drain').value
        params['smart_stamina_regen'] = self.get_parameter(
            'target_stamina_regen').value
        # tuning hook (tools/evader_compare.py --smart-json): a JSON object of
        # smart_<field> overrides in the environment wins over ROS params
        extra = os.environ.get('SMART_PARAMS_JSON')
        if extra:
            params.update(json.loads(extra))
        return SmartConfig.from_params(params)

    def _set_brain(self, kind):
        """Build the evader brain, preserving the live arena bounds so a
        mid-episode swap doesn't reset a shrinking arena's wall model."""
        if kind not in self.BRAINS:
            raise RuntimeError(
                f"unknown evader '{kind}'; expected one of "
                f"{'|'.join(self.BRAINS)}")
        if self.brain is not None and hasattr(self.brain, 'close'):
            self.brain.close()              # Nav2Evader: stop its goal loop
        bmin = self.brain.bmin if self.brain is not None \
            else self.get_parameter('bounds_min').value
        bmax = self.brain.bmax if self.brain is not None \
            else self.get_parameter('bounds_max').value
        if kind == 'nav2':
            # needs ROS (action clients / topics): hand it the node
            self.brain = Nav2Evader(
                bmin, bmax, self._obstacles, node=self,
                body_radius=self.get_parameter('target_body_radius').value)
        elif kind == 'smart':
            self.brain = SmartEvader(bmin, bmax, self._obstacles,
                                     self._smart_config())
        else:
            self.brain = self.BRAINS[kind](bmin, bmax, self._obstacles)
        self.kind = kind
        self._last_mode = None

    def _on_params(self, params):
        for p in params:
            if p.name != 'evader' or p.value == self.kind:
                continue
            if p.value not in self.BRAINS:
                return SetParametersResult(
                    successful=False,
                    reason=f"unknown evader '{p.value}'")
            self._set_brain(p.value)
            self.get_logger().info(f'evader brain -> {p.value}')
        return SetParametersResult(successful=True)

    def _now(self) -> float:
        """Node-clock seconds (follows /clock under use_sim_time)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_swarm_poses(self, msg: Float32MultiArray):
        d = msg.data                       # [N, x0,y0,th0,v0,w0, x1,...]
        now = self._now()
        for i in range(int(d[0])):
            b = 1 + 5 * i
            self.pursuers[i] = (d[b], d[b + 1], d[b + 2], now)

    def _on_bounds(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) >= 4:
            self.brain.bmin = [d[0], d[1]]
            self.brain.bmax = [d[2], d[3]]
            if hasattr(self.brain, 'geo'):          # AdaptiveEvader
                self.brain.geo.bmin = [d[0], d[1]]
                self.brain.geo.bmax = [d[2], d[3]]

    def _on_own_pose(self, msg: Pose):
        self.pose = (msg.x, msg.y, msg.theta)

    def control_cycle(self):
        if self.pose is None:
            return
        now = self._now()
        timeout = self.get_parameter('pose_timeout').value
        pursuers = [(x, y, th) for (x, y, th, t) in self.pursuers.values()
                    if now - t <= timeout]
        x, y, theta = self.pose
        ux, uy = self.brain.compute((x, y), theta, pursuers)
        # log behavior-mode switches tied to threat geometry (v4 D acceptance)
        mode = getattr(self.brain, 'mode', None)
        if mode is not None and mode != self._last_mode:
            self.get_logger().info(f'evader mode -> {mode}')
            self._last_mode = mode

        # Sprint only under threat; otherwise cruise at swarm speed so
        # stamina (if enabled in the sim) regenerates. This is what makes
        # a 2x target catchable-but-hard (§4.2): the swarm gets windows.
        near = min((math.hypot(px - x, py - y)
                    for (px, py, _t) in pursuers), default=1e9)
        base = self.get_parameter('agent_max_speed').value
        speed = (self._v_max()
                 if near < self.get_parameter('panic_distance').value
                 else base)
        # Steering uses the same deadlock-free conversion as the boids
        # (v3 §2), with the target's own w_max.
        v_lin, w_z = to_twist((ux * speed, uy * speed), theta,
                              kv=1.0, kw=3.0, v_min=0.4 * speed,
                              v_max=speed,
                              w_max=self.get_parameter(
                                  'target_omega_max').value,
                              last_w=self._last_w)
        blend = getattr(self.brain, 'blend_twist', None)
        if blend is not None:                 # Nav2Evader: mix in Nav2's cmd
            v_lin, w_z = blend(
                (v_lin, w_z), speed,
                self.get_parameter('target_omega_max').value,
                theta=theta, reactive_heading=math.atan2(uy, ux)
                if math.hypot(ux, uy) > 1e-6 else None,
                last_w=self._last_w)
        self._last_w = w_z
        twist = Twist()
        twist.linear.x = v_lin
        twist.angular.z = w_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
