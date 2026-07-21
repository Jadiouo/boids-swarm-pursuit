"""boid_controller_node — one instance per turtle (SDD v2 §4, §6.2, §7).

Control-loop architecture (§4.1): pose callbacks only write a cache;
a fixed-rate timer runs sense -> decide -> act and publishes one Twist.
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from . import behaviors


class CachedPose:
    __slots__ = ('x', 'y', 'theta', 'stamp')

    def __init__(self, x, y, theta, stamp):
        self.x, self.y, self.theta, self.stamp = x, y, theta, stamp


class BoidController(Node):

    def __init__(self):
        super().__init__('boid_controller')

        # --- Parameters (§5.1) -------------------------------------------
        self.declare_parameter('peers', ['turtle1'])       # all agent names
        self.declare_parameter('sensing_radius', 3.0)
        self.declare_parameter('safe_distance', 1.0)
        self.declare_parameter('w_separation', 1.5)
        self.declare_parameter('w_alignment', 1.0)
        self.declare_parameter('w_cohesion', 1.0)
        self.declare_parameter('w_boundary', 2.0)
        self.declare_parameter('w_wander', 0.5)
        self.declare_parameter('k_linear', 1.0)
        self.declare_parameter('k_angular', 4.0)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 5.0)
        self.declare_parameter('bounds_min', [0.5, 0.5])
        self.declare_parameter('bounds_max', [10.5, 10.5])
        self.declare_parameter('boundary_margin', 1.5)
        self.declare_parameter('wander_jitter', 0.3)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pose_timeout', 1.0)        # stale-cache expiry

        # Live tuning (§5.2)
        self.add_on_set_parameters_callback(self._on_params)

        # --- Identity & wiring (§3.1) ------------------------------------
        self.ns = self.get_namespace().strip('/')          # e.g. 'turtle3'
        if not self.ns:
            raise RuntimeError(
                'boid_controller must run inside a turtle namespace '
                '(e.g. ros2 run ... --ros-args -r __ns:=/turtle1)')

        self.cache: dict[str, CachedPose] = {}
        self.wander_angle = 0.0
        self._rng = random.Random()

        peers = list(self.get_parameter('peers').value)
        if self.ns not in peers:
            peers.append(self.ns)
        self._pose_subs = []
        for name in peers:   # own + peer poses go through the same cache
            self._pose_subs.append(self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, n=name: self._on_pose(msg, n), 10))

        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.control_cycle)

        self.get_logger().info(
            f'boid controller up for /{self.ns}, peers={peers}, '
            f'rate={rate} Hz')

    # --- Sensing (§4.1, §4.2): cheap callback, cache only ----------------
    def _on_pose(self, msg: Pose, name: str):
        self.cache[name] = CachedPose(msg.x, msg.y, msg.theta, time.monotonic())

    # --- Live parameter updates (§5.2) ------------------------------------
    def _on_params(self, params):
        for p in params:
            if p.name == 'control_rate_hz':
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False, reason='control_rate_hz must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / p.value,
                                               self.control_cycle)
        return SetParametersResult(successful=True)

    def _p(self, name):
        return self.get_parameter(name).value

    # --- Control cycle (§7) ------------------------------------------------
    def control_cycle(self):
        me = self.cache.get(self.ns)
        if me is None:
            return                                  # not spawned/sensed yet

        now = time.monotonic()
        timeout = self._p('pose_timeout')
        r = self._p('sensing_radius')
        me_xy = (me.x, me.y)

        neighbors = []
        for name, p in self.cache.items():
            if name == self.ns:
                continue
            if now - p.stamp > timeout:             # expire stale peers
                continue
            if math.hypot(p.x - me.x, p.y - me.y) < r:
                neighbors.append((p.x, p.y, p.theta))

        if neighbors:                               # Flocking
            sx, sy = behaviors.separation(me_xy, neighbors,
                                          self._p('safe_distance'))
            ax, ay = behaviors.alignment(neighbors)
            cx, cy = behaviors.cohesion(me_xy, neighbors)
            vx = (self._p('w_separation') * sx
                  + self._p('w_alignment') * ax
                  + self._p('w_cohesion') * cx)
            vy = (self._p('w_separation') * sy
                  + self._p('w_alignment') * ay
                  + self._p('w_cohesion') * cy)
        else:                                       # Wander fallback (§4.3.5)
            jitter = self._p('wander_jitter')
            self.wander_angle = behaviors.wrap_angle(
                self.wander_angle + self._rng.uniform(-jitter, jitter))
            wx, wy = behaviors.wander(me.theta, self.wander_angle)
            vx = self._p('w_wander') * wx
            vy = self._p('w_wander') * wy

        # Boundary avoidance is always considered (§4.6: highest priority)
        bx, by = behaviors.boundary(me_xy, self._p('bounds_min'),
                                    self._p('bounds_max'),
                                    self._p('boundary_margin'))
        vx += self._p('w_boundary') * bx
        vy += self._p('w_boundary') * by

        v_lin, w_z = behaviors.to_twist(
            (vx, vy), me.theta,
            self._p('k_linear'), self._p('k_angular'),
            self._p('v_min'), self._p('v_max'), self._p('w_max'))

        twist = Twist()
        twist.linear.x = v_lin
        twist.angular.z = w_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BoidController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
