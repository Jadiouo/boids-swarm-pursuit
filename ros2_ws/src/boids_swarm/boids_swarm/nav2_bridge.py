"""Nav2 plumbing for the target (M7 spike, SDD v3 §6.4).

Adapts the pygame sim's topics to what Nav2 expects, without touching the
sim: /target/pose -> /target/odom + TF (map->odom identity, odom->base_link),
circular obstacles + arena boundary -> /map (OccupancyGrid, transient_local),
boid positions -> /boids_cloud (PointCloud2, MARKING) and /boids_scan
(LaserScan, CLEARING only) for the costmap obstacle layer. The scan is a
synthetic 360-degree lidar that sees only boids: every ray the boids do not
block reports "free" out to scan_range_max, so cells a boid has just left are
raytraced clear instead of lingering as phantom obstacles. Runs on sim time.
"""

import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

from boids_swarm.occupancy import rasterize_arena


def _yaw_quat(theta):
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


class Nav2Bridge(Node):

    def __init__(self):
        super().__init__('nav2_bridge')
        self.declare_parameter('world_size', 20.0)
        self.declare_parameter('map_resolution', 0.1)
        # The wall is drawn OUTSIDE the arena (grid padded by border_cells),
        # so the lethal band starts exactly at the real wall surface and the
        # body-touching position (centre 0.15 m off the wall) is a high-cost
        # but traversable cell. See occupancy.rasterize_arena.
        self.declare_parameter('border_cells', 1)
        self.declare_parameter('obstacles', [0.0])      # flat [x, y, r, ...]
        self.declare_parameter('cloud_rate_hz', 10.0)
        # live switch: False publishes EMPTY clouds (no boids), so tests can
        # check boid cost really comes from the cloud. (Silence would not do:
        # with observation_keep_time 0 the obstacle layer keeps re-marking the
        # last cloud it received.)
        self.declare_parameter('publish_cloud', True)
        self.declare_parameter('boid_cloud_radius_pts', 8)   # points per boid ring
        self.declare_parameter('boid_body_radius', 0.25)
        self.declare_parameter('scan_rays', 360)
        self.declare_parameter('scan_range_max', 20.0)

        self.tf_pub = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/target/odom', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '/boids_cloud', 5)
        self.scan_pub = self.create_publisher(LaserScan, '/boids_scan', 5)
        self.create_subscription(Pose, '/target/pose', self._on_pose, 10)
        self.create_subscription(Float32MultiArray, '/swarm/poses',
                                 self._on_swarm, 10)
        self.boids = []
        self.pose = None                  # (x, y, theta) of the target
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(1.0 / float(self.get_parameter('cloud_rate_hz').value),
                          self._publish_cloud)
        self._publish_map()

    def _obstacles(self, flat=None):
        if flat is None:
            flat = self.get_parameter('obstacles').value
        flat = [float(v) for v in flat]
        return [(flat[i], flat[i + 1], flat[i + 2])
                for i in range(0, len(flat) - 2, 3)] if len(flat) >= 3 else []

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'obstacles':
                self._publish_map(self._obstacles(p.value))
        return SetParametersResult(successful=True)

    def _publish_map(self, obstacles=None):
        if obstacles is None:
            obstacles = self._obstacles()
        w = float(self.get_parameter('world_size').value)
        res = float(self.get_parameter('map_resolution').value)
        g = rasterize_arena(w, w, res, obstacles,
                            int(self.get_parameter('border_cells').value))
        m = OccupancyGrid()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.info = MapMetaData()
        m.info.resolution = g.resolution
        m.info.width, m.info.height = g.width, g.height
        m.info.origin.position.x, m.info.origin.position.y = g.origin
        m.info.origin.orientation.w = 1.0
        m.data = g.data
        self.map_pub.publish(m)
        self.get_logger().info(
            f'/map published: {g.width}x{g.height} @ {res} m, '
            f'{len(obstacles)} circular obstacles')

    def _on_pose(self, p):
        self.pose = (p.x, p.y, p.theta)
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _yaw_quat(p.theta)
        tfs = []
        t0 = TransformStamped()
        t0.header.stamp, t0.header.frame_id = stamp, 'map'
        t0.child_frame_id = 'odom'
        t0.transform.rotation.w = 1.0
        tfs.append(t0)
        t1 = TransformStamped()
        t1.header.stamp, t1.header.frame_id = stamp, 'odom'
        t1.child_frame_id = 'base_link'
        t1.transform.translation.x, t1.transform.translation.y = p.x, p.y
        t1.transform.rotation.z, t1.transform.rotation.w = qz, qw
        tfs.append(t1)
        self.tf_pub.sendTransform(tfs)
        o = Odometry()
        o.header.stamp, o.header.frame_id = stamp, 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = p.x, p.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = p.linear_velocity
        o.twist.twist.angular.z = p.angular_velocity
        self.odom_pub.publish(o)

    def _on_swarm(self, msg):
        d = msg.data
        n = int(d[0]) if d else 0
        self.boids = [(d[1 + 5 * i], d[2 + 5 * i]) for i in range(n)]

    def _publish_cloud(self):
        """Each boid becomes a small ring of points (body radius) so the
        costmap obstacle layer marks a disc, not a single cell. The cloud is
        expressed in base_link (like a real sensor) so the sensor origin is
        the robot, which lies inside the rolling window (raytrace clearing)."""
        if self.pose is None:
            return
        px, py, th = self.pose
        c_, s_ = math.cos(th), math.sin(th)
        k = int(self.get_parameter('boid_cloud_radius_pts').value)
        r = float(self.get_parameter('boid_body_radius').value)
        pts = []
        show = bool(self.get_parameter('publish_cloud').value)
        for (wx, wy) in (self.boids if show else []):
            dx, dy = wx - px, wy - py
            x, y = c_ * dx + s_ * dy, -s_ * dx + c_ * dy
            pts.append((x, y))
            pts += [(x + r * math.cos(2 * math.pi * j / k),
                     y + r * math.sin(2 * math.pi * j / k)) for j in range(k)]
        c = PointCloud2()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = 'base_link'
        c.height, c.width = 1, len(pts)
        c.fields = [PointField(name=n, offset=4 * i,
                               datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate('xyz')]
        c.is_bigendian, c.point_step = False, 12
        c.row_step = 12 * len(pts)
        c.is_dense = True
        c.data = b''.join(struct.pack('<fff', x, y, 0.1) for x, y in pts)
        self.cloud_pub.publish(c)
        self.scan_pub.publish(self._make_scan(px, py, th, c.header.stamp))

    def _make_scan(self, px, py, th, stamp):
        n = int(self.get_parameter('scan_rays').value)
        rmax = float(self.get_parameter('scan_range_max').value)
        rb = float(self.get_parameter('boid_body_radius').value)
        c_, s_ = math.cos(th), math.sin(th)
        local = [(c_ * (wx - px) + s_ * (wy - py),
                  -s_ * (wx - px) + c_ * (wy - py)) for (wx, wy) in self.boids]
        free = rmax - 0.05          # < range_max: kept by the projector
        ranges = []
        inc = 2.0 * math.pi / n
        for k in range(n):
            a = -math.pi + k * inc
            ux, uy = math.cos(a), math.sin(a)
            best = free
            for (bx, by) in local:
                proj = bx * ux + by * uy
                if proj <= 0.0:
                    continue
                perp2 = bx * bx + by * by - proj * proj
                if perp2 < rb * rb:
                    best = min(best, proj - math.sqrt(rb * rb - perp2))
            ranges.append(max(best, 0.06))
        m = LaserScan()
        m.header.stamp = stamp
        m.header.frame_id = 'base_link'
        m.angle_min, m.angle_max = -math.pi, -math.pi + (n - 1) * inc
        m.angle_increment = inc
        m.range_min, m.range_max = 0.05, rmax
        m.ranges = ranges
        return m


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
