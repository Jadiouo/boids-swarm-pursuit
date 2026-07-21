"""swarm_spawner_node — spawn N turtles at valid random poses (SDD v2 §6.1).

Optionally kills the default turtle1, then spawns turtle1..turtleN at
random positions inside [bounds_min + margin, bounds_max - margin] so
nobody starts inside a wall.
"""

import math
import random

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill


class SwarmSpawner(Node):

    def __init__(self):
        super().__init__('swarm_spawner')
        self.declare_parameter('num_turtles', 6)
        self.declare_parameter('kill_default_turtle', True)
        self.declare_parameter('bounds_min', [0.5, 0.5])
        self.declare_parameter('bounds_max', [10.5, 10.5])
        self.declare_parameter('boundary_margin', 1.5)
        self.declare_parameter('seed', -1)  # -1 = nondeterministic

        self.spawn_cli = self.create_client(Spawn, 'spawn')
        self.kill_cli = self.create_client(Kill, 'kill')

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def run(self):
        n = self.get_parameter('num_turtles').value
        bmin = self.get_parameter('bounds_min').value
        bmax = self.get_parameter('bounds_max').value
        margin = self.get_parameter('boundary_margin').value
        seed = self.get_parameter('seed').value
        rng = random.Random(seed if seed >= 0 else None)

        while not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('waiting for /spawn service...')

        if self.get_parameter('kill_default_turtle').value:
            result = self._call(self.kill_cli, Kill.Request(name='turtle1'))
            if result is None:
                self.get_logger().warn('kill turtle1 failed (may not exist)')
            else:
                self.get_logger().info('killed default turtle1')

        x_lo, x_hi = bmin[0] + margin, bmax[0] - margin
        y_lo, y_hi = bmin[1] + margin, bmax[1] - margin
        for i in range(1, n + 1):
            req = Spawn.Request()
            req.name = f'turtle{i}'
            req.x = rng.uniform(x_lo, x_hi)
            req.y = rng.uniform(y_lo, y_hi)
            req.theta = rng.uniform(-math.pi, math.pi)
            result = self._call(self.spawn_cli, req)
            if result is None:
                self.get_logger().error(f'spawn {req.name} failed')
            else:
                self.get_logger().info(
                    f'spawned {result.name} at '
                    f'({req.x:.2f}, {req.y:.2f}, θ={req.theta:.2f})')

        self.get_logger().info(f'spawner done: {n} turtles')


def main(args=None):
    rclpy.init(args=args)
    node = SwarmSpawner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
