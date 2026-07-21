"""target_controller_node — the evader (SDD v3 §4.2–4.3, §6.3).

Pluggable brain behind the `evader` parameter: `reactive` (default) or
`nav2` (M7 stretch). Publishes /target/cmd_vel; the sim enforces the
2x speed cap and the turn-rate limit that keeps the game winnable.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from turtlesim.msg import Pose

from .geometry import to_twist
from .behaviors.evasion import (AdaptiveEvader, Nav2Evader, ReactiveEvader)


class TargetController(Node):

    def __init__(self):
        super().__init__('target_controller')
        self.declare_parameter('num_agents', 12)
        self.declare_parameter('evader', 'reactive')      # reactive | nav2
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

        n = int(self.get_parameter('num_agents').value)
        flat = [float(v) for v in self.get_parameter('obstacles').value]
        obstacles = [tuple(flat[i:i + 3])
                     for i in range(0, len(flat) - 2, 3)]

        kind = self.get_parameter('evader').value
        brain_cls = {'reactive': ReactiveEvader, 'adaptive': AdaptiveEvader,
                     'nav2': Nav2Evader}[kind]
        self.kind = kind
        self.brain = brain_cls(self.get_parameter('bounds_min').value,
                               self.get_parameter('bounds_max').value,
                               obstacles)
        self._last_mode = None

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
        self.v_max = (self.get_parameter('agent_max_speed').value
                      * self.get_parameter('target_speed_multiplier').value)
        self.w_max = self.get_parameter('target_omega_max').value
        self._last_w = 0.0
        self.get_logger().info(f'target up: evader={kind}, v_max={self.v_max}')

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
        speed = (self.v_max
                 if near < self.get_parameter('panic_distance').value
                 else base)
        # Steering uses the same deadlock-free conversion as the boids
        # (v3 §2), with the target's own w_max.
        v_lin, w_z = to_twist((ux * speed, uy * speed), theta,
                              kv=1.0, kw=3.0, v_min=0.4 * speed,
                              v_max=speed, w_max=self.w_max,
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
