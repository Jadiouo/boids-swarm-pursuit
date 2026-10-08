"""Headless Nav2 evader plumbing checks (M7). Run explicitly after build:

    cd ros2_ws && colcon build --symlink-install --base-paths src ...
    source install/setup.bash        # + pygame on PYTHONPATH
    SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy ROS_DOMAIN_ID=210 \\
        python3 -m pytest -q src/boids_swarm/ros_test/test_nav2_ros_integration.py

(a) /map, TF map->odom->base_link and /target/odom exist and are stamped
    with the sim's /clock (not wall time);
(b) boids appear as obstacles in BOTH the local and the global costmap at
    their position at the costmap's own stamp, and the cost disappears when
    the marking cloud is switched off (negative control);
(c) with no boids the target is driven around obstacles to a goal via
    ComputePathToPose + FollowPath without touching any obstacle;
(d) the launch gives sim, controllers and Nav2 bridge identical obstacles;
(e) a wall-hugging / pocket start (sanity seed 3, (3.23, 0.31)) plans;
(f) Nav2Evader fallback: an unreachable goal is counted, blacklisted, not
    re-sent, the mode drops to reactive, and a late/superseded FollowPath
    result is not counted as a failure.
Two Nav2 stacks are brought up (~15 s each); total well under 2 minutes.
"""

import importlib.util
import math
import os
import signal
import subprocess
import threading
import time
from collections import deque

import pytest
import rclpy
from launch import LaunchContext
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
from turtlesim.msg import Pose
from lifecycle_msgs.srv import GetState

BODY_R = 0.15
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, '..'))


class Probe(Node):
    def __init__(self):
        super().__init__('nav2_it_probe', parameter_overrides=[])
        self.pose = None
        self.boids = []
        self.clock = None
        self.odom = None
        self.map = None
        self.costmap = None
        self.gcostmap = None
        self.boid_hist = deque(maxlen=400)       # (sim time, [(x, y)])
        self.samples = []
        self.recording = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Pose, '/target/pose', self._cb_pose, 10)
        self.create_subscription(Float32MultiArray, '/swarm/poses',
                                 self._cb_swarm, 10)
        self.create_subscription(Clock, '/clock', self._cb_clock, 10)
        self.create_subscription(Odometry, '/target/odom', self._cb_odom, 10)
        self.create_subscription(OccupancyGrid, '/map', self._cb_map, latched)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 self._cb_costmap, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._cb_gcostmap, latched)
        self.plan_ac = ActionClient(self, ComputePathToPose,
                                    'compute_path_to_pose')
        self.follow_ac = ActionClient(self, FollowPath, 'follow_path')
        self.sim_params = self.create_client(SetParameters,
                                             '/pygame_sim/set_parameters')
        self.bridge_params = self.create_client(
            SetParameters, '/nav2_bridge/set_parameters')

    def clear_costmap(self, name):
        """Wipe the obstacle layer of '<name>_costmap' (global|local)."""
        cli = self.create_client(
            ClearEntireCostmap,
            f'/{name}_costmap/clear_entirely_{name}_costmap')
        try:
            assert cli.wait_for_service(10.0), 'clear_entirely service'
            fut = cli.call_async(ClearEntireCostmap.Request())
            self.wait_for(fut.done, 10, 'clear costmap')
        finally:
            self.destroy_client(cli)

    def set_bridge_bool(self, name, value):
        assert self.bridge_params.wait_for_service(10.0)
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(value)))]
        fut = self.bridge_params.call_async(req)
        self.wait_for(fut.done, 10, f'set {name}')
        assert fut.result().results[0].successful

    def _cb_pose(self, p):
        self.pose = p
        if self.recording:
            self.samples.append((self.clock or 0.0, p.x, p.y, p.linear_velocity))

    def _cb_swarm(self, m):
        d = m.data
        n = int(d[0]) if len(d) else 0
        self.boids = [(d[1 + 5 * i], d[2 + 5 * i]) for i in range(n)]
        if self.clock is not None:
            self.boid_hist.append((self.clock, list(self.boids)))

    def boids_at(self, t):
        """Boid positions of the snapshot closest in sim time to t."""
        if not self.boid_hist:
            return []
        return min(self.boid_hist, key=lambda h: abs(h[0] - t))[1]

    def _cb_gcostmap(self, m):
        self.gcostmap = m

    def _cb_clock(self, m):
        self.clock = m.clock.sec + m.clock.nanosec * 1e-9

    def _cb_odom(self, m):
        self.odom = m

    def _cb_map(self, m):
        self.map = m

    def _cb_costmap(self, m):
        self.costmap = m

    def wait_for(self, cond, timeout, what):
        t0 = time.monotonic()
        while not cond():
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(what)
            time.sleep(0.02)

    def lifecycle_active(self, name):
        cli = self.create_client(GetState, f'/{name}/get_state')
        try:
            if not cli.service_is_ready():
                return False
            fut = cli.call_async(GetState.Request())
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 1.0:
                time.sleep(0.01)
            return fut.done() and fut.result().current_state.id == 3
        finally:
            self.destroy_client(cli)

    def set_obstacles(self, obstacles):
        flat = [float(v) for o in obstacles for v in o] or [0.0]
        for cli in (self.sim_params, self.bridge_params):
            assert cli.wait_for_service(10.0)
            req = SetParameters.Request()
            req.parameters = [Parameter(name='obstacles', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=flat))]
            fut = cli.call_async(req)
            self.wait_for(fut.done, 10, 'set obstacles')
            assert fut.result().results[0].successful


def _spawn(cmd, log):
    f = open(log, 'w')
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                            start_new_session=True)


def _stop(proc):
    if proc and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


class Stack:
    """pygame_sim (headless) + Nav2 stack + probe node."""

    def __init__(self, tmp_path, agents, seed):
        self.sim = self.nav = self.node = self.ex = None
        try:
            self._start(tmp_path, agents, seed)
        except BaseException:
            self.close()                  # never leave orphaned sim/Nav2
            raise

    def _start(self, tmp_path, agents, seed):
        domain = os.environ.get('ROS_DOMAIN_ID', '210')
        assert int(domain) <= 232, 'ROS_DOMAIN_ID must be <= 232'
        os.environ['ROS_DOMAIN_ID'] = domain
        self.sim = _spawn(
            ['ros2', 'run', 'boids_swarm', 'pygame_sim', '--ros-args',
             '-p', 'headless:=true', '-p', 'ui_enabled:=false',
             '-p', f'num_agents:={agents}', '-p', f'seed:={seed}',
             '-p', 'time_scale:=1.0',
             '-p', 'target_speed_multiplier:=1.8',
             '-p', 'target_omega_max:=1.2',
             '-p', f'target_body_radius:={BODY_R}',
             '-p', 'target_stamina_enabled:=false',
             '-p', 'episode_time_limit:=0.0', '-p', 'auto_reset:=false'],
            tmp_path / f'sim_{agents}.log')
        self.nav = _spawn(
            ['ros2', 'launch', 'boids_swarm', 'nav2_target.launch.py'],
            tmp_path / f'nav2_{agents}.log')
        if not rclpy.ok():
            rclpy.init()
        self.node = Probe()
        self.ex = MultiThreadedExecutor(num_threads=3)
        self.ex.add_node(self.node)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()
        n = self.node
        n.wait_for(lambda: n.pose is not None and n.clock is not None, 120,
                   'sim pose/clock')
        n.wait_for(lambda: n.lifecycle_active('planner_server')
                   and n.lifecycle_active('controller_server'), 300,
                   'nav2 lifecycle active')
        n.wait_for(lambda: n.plan_ac.wait_for_server(0.1)
                   and n.follow_ac.wait_for_server(0.1), 60, 'action servers')
        n.wait_for(lambda: n.map is not None and n.costmap is not None
                   and n.gcostmap is not None and n.odom is not None, 60,
                   'map/costmap/odom')
        if agents:
            n.wait_for(lambda: len(n.boids) == agents, 30, '/swarm/poses')

    def close(self):
        if self.ex is not None:
            self.ex.shutdown()
        if self.node is not None:
            self.node.destroy_node()
        _stop(self.nav)
        _stop(self.sim)
        time.sleep(1.0)


# Class-scoped so the two stacks never coexist on one ROS domain.
@pytest.fixture(scope='class')
def stack_empty(tmp_path_factory):
    s = Stack(tmp_path_factory.mktemp('nav2_empty'), agents=0, seed=7)
    yield s
    s.close()


@pytest.fixture(scope='class')
def stack_boids(tmp_path_factory):
    s = Stack(tmp_path_factory.mktemp('nav2_boids'), agents=30, seed=7)
    yield s
    s.close()


# ---------------------------------------------------------------- (b)
def _cell_cost(cm, x, y):
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    ix = int(math.floor((x - ox) / cm.info.resolution))
    iy = int(math.floor((y - oy) / cm.info.resolution))
    if not (0 <= ix < cm.info.width and 0 <= iy < cm.info.height):
        return None
    return cm.data[iy * cm.info.width + ix]


# ---------------------------------------------------------------- (c)
def _course(sx, sy):
    corners = [(3.5, 3.5), (16.5, 3.5), (3.5, 16.5), (16.5, 16.5)]
    gx, gy = max(corners, key=lambda c: math.hypot(c[0] - sx, c[1] - sy))
    d = math.hypot(gx - sx, gy - sy)
    ux, uy = (gx - sx) / d, (gy - sy) / d
    nx, ny = -uy, ux
    obs = [(sx + ux * d * f + nx * s, sy + uy * d * f + ny * s, 1.5)
           for f, s in ((0.3, 0.8), (0.55, -0.8), (0.75, 0.8))]
    obs = [o for o in obs if math.hypot(o[0] - sx, o[1] - sy) > o[2] + 1.0
           and math.hypot(o[0] - gx, o[1] - gy) > o[2] + 1.0]
    return (gx, gy), obs


class TestNoBoids:
    def test_map_tf_odom_exist_and_follow_sim_clock(self, stack_empty):
        n = stack_empty.node
        # 200x200 arena + a 1-cell wall drawn OUTSIDE it (origin -0.1)
        assert (n.map.info.width, n.map.info.height) == (202, 202)
        assert n.map.info.resolution == pytest.approx(0.1)
        assert n.map.info.origin.position.x == pytest.approx(-0.1)
        assert n.map.header.frame_id == 'map'
        assert n.map.data[0] == 100 and n.map.data[101 * 202 + 101] == 0
        assert _cell_cost(n.map, 0.05, 10.0) == 0      # arena floor at the wall
        n.wait_for(lambda: n.tf_buffer.can_transform(
            'map', 'base_link', rclpy.time.Time()), 10, 'TF map->base_link')
        tf = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        p = n.pose
        assert tf.transform.translation.x == pytest.approx(p.x, abs=0.5)
        assert tf.transform.translation.y == pytest.approx(p.y, abs=0.5)
        clk = n.clock
        t_tf = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        o = n.odom.header.stamp
        t_odom = o.sec + o.nanosec * 1e-9
        # sim time is small (seconds since sim start); wall time would be ~1.7e9
        assert clk < 1e6
        assert abs(t_odom - clk) < 1.0
        assert abs(t_tf - clk) < 1.0
        assert n.odom.header.frame_id == 'odom'
        assert n.odom.child_frame_id == 'base_link'



    def test_follows_path_around_obstacles_without_boids(self, stack_empty):
        n = stack_empty.node
        p = n.pose
        goal, obs = _course(p.x, p.y)
        assert len(obs) >= 2
        n.set_obstacles(obs)
        time.sleep(3.0)                       # map -> both costmaps
        plan = ComputePathToPose.Goal()
        plan.goal.header.frame_id = 'map'
        plan.goal.header.stamp = n.get_clock().now().to_msg()
        plan.goal.pose.position.x, plan.goal.pose.position.y = goal
        plan.goal.pose.orientation.w = 1.0
        plan.use_start = False
        plan.planner_id = 'GridBased'
        f = n.plan_ac.send_goal_async(plan)
        n.wait_for(f.done, 10, 'plan accepted')
        r = f.result().get_result_async()
        n.wait_for(r.done, 10, 'plan result')
        assert r.result().status == 4
        path = r.result().result.path
        assert len(path.poses) > 10
        n.samples.clear()
        n.recording = True
        fg = FollowPath.Goal()
        fg.path, fg.controller_id = path, 'FollowPath'
        fg.goal_checker_id = 'general_goal_checker'
        ff = n.follow_ac.send_goal_async(fg)
        n.wait_for(ff.done, 10, 'follow accepted')
        gh = ff.result()
        assert gh.accepted
        rf = gh.get_result_async()
        try:
            n.wait_for(rf.done, 45, 'follow result')
        finally:
            n.recording = False
        assert rf.result().status == 4, 'FollowPath did not succeed'
        end = n.samples[-1]
        assert math.hypot(end[1] - goal[0], end[2] - goal[1]) < 0.7
        clear = min(math.hypot(s[1] - cx, s[2] - cy) - r_ - BODY_R
                    for s in n.samples for (cx, cy, r_) in obs)
        assert clear > 0.02, f'touched an obstacle (clearance {clear:.3f} m)'
        assert max(s[3] for s in n.samples) <= 3.6 + 1e-3     # params-derived cap




def _max_cost_near(cm, x, y, r):
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    res, w, h = cm.info.resolution, cm.info.width, cm.info.height
    best = None
    k = int(math.ceil(r / res))
    cx, cy = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
    for iy in range(cy - k, cy + k + 1):
        for ix in range(cx - k, cx + k + 1):
            if 0 <= ix < w and 0 <= iy < h and \
                    math.hypot((ix + .5) * res + ox - x,
                               (iy + .5) * res + oy - y) <= r:
                c = cm.data[iy * w + ix]
                best = c if best is None else max(best, c)
    return best


def _stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def _fresh(n, attr, timeout=8.0):
    """Next costmap message published after now."""
    old = getattr(n, attr)
    t0 = _stamp(old) if old is not None else -1.0
    n.wait_for(lambda: getattr(n, attr) is not None
               and _stamp(getattr(n, attr)) > t0, timeout, attr)
    return getattr(n, attr)


class TestWithBoids:
    def _boid_costs(self, n, attr, window):
        """Max cost within 0.35 m of each boid, at the costmap's own stamp
        (not 3 s earlier: boids move 2 m/s), for boids inside `window` m of
        the target."""
        cm = _fresh(n, attr)
        p = n.pose
        out = []
        for (bx, by) in n.boids_at(_stamp(cm)):
            if math.hypot(bx - p.x, by - p.y) < window:
                out.append(_max_cost_near(cm, bx, by, 0.35))
        return out

    def test_boids_are_dynamic_obstacles_in_local_and_global_costmap(
            self, stack_boids):
        n = stack_boids.node
        n.set_bridge_bool('publish_cloud', True)
        time.sleep(2.0)
        for attr, window in (('costmap', 5.0), ('gcostmap', 8.0)):
            costs = self._boid_costs(n, attr, window)
            assert costs, f'fixture seed has no boid within {window} m'
            marked = [c for c in costs if c is not None and c >= 99]
            assert len(marked) >= max(1, len(costs) // 2), \
                f'{attr}: boid cells cost {costs}'

    def test_negative_control_no_cloud_no_boid_cost(self, stack_boids):
        """Boid cost must come from the cloud: wipe the costmap with the cloud
        off and the boid cells stay free; switch it on and they come back.
        (The fixture's boids are stationary, so a plain 'switch it off' would
        leave the old marks in place and prove nothing.)"""
        n = stack_boids.node
        n.set_bridge_bool('publish_cloud', True)
        time.sleep(2.0)
        with_cloud = self._boid_costs(n, 'gcostmap', 8.0)
        n.set_bridge_bool('publish_cloud', False)
        try:
            time.sleep(1.0)
            n.clear_costmap('global')
            time.sleep(1.0)
            without = self._boid_costs(n, 'gcostmap', 8.0)
        finally:
            n.set_bridge_bool('publish_cloud', True)
        time.sleep(2.5)
        back = self._boid_costs(n, 'gcostmap', 8.0)
        hi = sum(1 for c in with_cloud if c is not None and c >= 99)
        lo = sum(1 for c in without if c is not None and c >= 99)
        again = sum(1 for c in back if c is not None and c >= 99)
        assert hi >= 1 and again >= 1
        assert lo == 0, (with_cloud, without)


# ---------------------------------------------------------------- (e)
S3_START = (3.23, 0.31)       # the pocket start behind 15 of 45 sanity failures


def _plan(n, start, goal, timeout=10):
    g = ComputePathToPose.Goal()
    g.goal.header.frame_id = 'map'
    g.goal.header.stamp = n.get_clock().now().to_msg()
    g.goal.pose.position.x, g.goal.pose.position.y = goal
    g.goal.pose.orientation.w = 1.0
    g.use_start = True
    g.start.header = g.goal.header
    g.start.pose.position.x, g.start.pose.position.y = start
    g.start.pose.orientation.w = 1.0
    g.planner_id = 'GridBased'
    f = n.plan_ac.send_goal_async(g)
    n.wait_for(f.done, timeout, 'plan accepted')
    r = f.result().get_result_async()
    n.wait_for(r.done, timeout, 'plan result')
    return r.result().status, r.result().result.path


class TestPocketStart:
    def test_wall_hugging_and_s3_pocket_starts_plan(self, stack_empty):
        from boids_swarm.world_gen import WorldGenerator
        n = stack_empty.node
        obs = [tuple(o) for o in WorldGenerator(3, 20.0).generate(
            'obstacle_field').obstacles]
        n.set_obstacles(obs)
        time.sleep(3.0)
        gm = _fresh(n, 'gcostmap')
        # the global costmap must price the legal wall-hugging cell, not block it
        spots = [(BODY_R, 10.0), (10.0, BODY_R), (19.85, 10.0),
                 (10.0, 19.85), S3_START]
        spots = [q for q in spots if all(math.hypot(q[0] - cx, q[1] - cy)
                                         > r + 1.5 for (cx, cy, r) in obs)
                 or q == S3_START]
        assert len(spots) >= 2
        for (x, y) in spots:
            c = _cell_cost(gm, x, y)
            # OccupancyGrid scale: 100 lethal, 99 inscribed, 1..98 traversable
            assert c is not None and 0 < c < 99, (x, y, c)
        goals = [g for g in ((10.0, 10.0), (12.0, 4.0), (5.0, 12.0),
                             (14.0, 14.0), (17.0, 8.0))
                 if all(math.hypot(g[0] - cx, g[1] - cy) > r + 1.0
                        for (cx, cy, r) in obs)]
        assert len(goals) >= 2
        for goal in goals:
            # raw sim positions, NOT the shifted plan_start
            for start in (S3_START, (BODY_R, 10.0)):
                status, path = _plan(n, start, goal)
                assert status == 4 and len(path.poses) > 5, \
                    (start, goal, status)
        n.set_obstacles([])


# ---------------------------------------------------------------- (f)
class TestEvaderFallback:
    def _evader(self, n, obstacles, goal_fn):
        """A real Nav2Evader on the probe node whose goal picker is replaced
        by a scripted one (honouring the blacklist like the real one)."""
        from boids_swarm.behaviors import nav2_evader as ne
        ev = ne.Nav2Evader([0.0, 0.0], [20.0, 20.0], obstacles, node=n,
                           body_radius=BODY_R)

        def pick(rng, self_xy, pursuers, obs, bmin, bmax, inc, cfg,
                 emap=None, blocked=None):
            g = goal_fn()
            if blocked is not None and blocked(g):
                return tuple(self_xy), -math.inf
            return g, 0.0
        return ev, ne, pick

    def _drive(self, n, ev, seconds):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            p = n.pose
            ev.compute((p.x, p.y), p.theta, [])
            time.sleep(0.05)

    def test_unreachable_goal_is_counted_blacklisted_and_not_resent(
            self, stack_empty, monkeypatch):
        n = stack_empty.node
        ring = [(10.0 + 3.0 * math.cos(a), 10.0 + 3.0 * math.sin(a), 0.7)
                for a in [2 * math.pi * k / 40 for k in range(40)]]
        n.set_obstacles(ring)
        time.sleep(3.0)
        ev, ne, pick = self._evader(n, ring, lambda: (10.0, 10.0))
        ev.blacklist.ttl = 60.0          # outlast the observation window
        monkeypatch.setattr(ne, 'select_escape_goal', pick)
        try:
            t0 = time.monotonic()
            while ev.stats['plan_fail'] < 1 and time.monotonic() - t0 < 15:
                self._drive(n, ev, 0.2)
            st = dict(ev.stats)
            assert st['plan_fail'] >= 1, st
            assert st['plan_ok'] == 0
            assert ev.blacklist.blocked((10.0, 10.0), ev._now())
            assert ev.goal is None                  # failed goal dropped
            ev.blend_twist((1.0, 0.0), 2.0, 1.2, theta=n.pose.theta)
            assert ev.mode == 'reactive'
            # nothing is re-sent for the blacklisted goal, neither during
            # the hold nor after it (at most the one request still in flight)
            self._drive(n, ev, 1.0 + 3.0)
            assert ev.stats['plan_requests'] <= st['plan_fail'] + 1, ev.stats
            assert ev.stats['plan_requests'] == st['plan_requests'], ev.stats
            assert ev.stats['late_dropped'] <= 1
        finally:
            ev.close()
            n.set_obstacles([])
            time.sleep(1.0)

    def test_preempted_follow_is_not_a_failure(self, stack_empty,
                                               monkeypatch):
        n = stack_empty.node
        n.set_obstacles([])
        time.sleep(3.0)
        # the very first ComputePathToPose after startup can take > the 2 s
        # plan_timeout (cold planner); warm it up so this test is about preempt
        assert _plan(n, (5.0, 5.0), (12.0, 12.0), timeout=20)[0] == 4
        seq = [(15.0, 4.0), (4.0, 15.0)]
        state = {'t0': time.monotonic()}

        def goal():
            return seq[int((time.monotonic() - state['t0']) / 3.0) % 2]
        ev, ne, pick = self._evader(n, [], goal)
        monkeypatch.setattr(ne, 'select_escape_goal', pick)
        try:
            self._drive(n, ev, 10.0)
        finally:
            ev.close()
        st = ev.stats
        assert st['plan_ok'] >= 2, st
        # a loaded CI box may time a plan out; that is a counted plan_fail, but
        # a preempted FollowPath must never be counted as an abort
        assert st['follow_abort'] == 0, st
        assert st['plan_fail'] == st['plan_timeout'], st
        assert st['goals'] >= 2


# ---------------------------------------------------------------- (d)
def _load_launch(name):
    spec = importlib.util.spec_from_file_location(
        name.replace('.', '_'), os.path.join(PKG, 'launch', name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _launch_actions(env, seed):
    mod = _load_launch('pursuit.launch.py')
    ctx = LaunchContext()
    defaults = {'num_agents': '4', 'strategy': 'intercept', 'game_mode': 'ai',
                'seed': str(seed), 'headless': 'true', 'trails': 'false',
                'capture_mode': 'hull', 'episodes_max': '1',
                'time_limit': '30', 'stamina': 'true', 'obstacles': '',
                'evader': 'nav2', 'perception': 'perfect', 'env': env,
                'shrink_rate': '0.0', 'ui': 'false', 'screenshot_dir': '',
                'screenshot_period': '5', 'sharing_mode': 'legacy',
                'nav2_config': 'nav2_target.yaml', 'time_scale': '', 'warmup': '0.0',
                'pursuer_delay': '0.0', 'shared_sighting_qos_depth': '',
                'oracle_max_hops': '', 'relay_log_dir': ''}
    ctx.launch_configurations.update(defaults)
    return ctx, mod.launch_setup(ctx)


def _param(node, name):
    for params in node._Node__parameters:
        if isinstance(params, dict):
            for k, v in params.items():
                if ''.join(getattr(x, 'text', str(x)) for x in k) == name:
                    return v
    return None


@pytest.mark.parametrize('env,seed', [('obstacle_field', 3),
                                      ('obstacle_field', 7), ('pillar', 5)])
def test_launch_gives_sim_target_and_bridge_the_same_obstacles(env, seed):
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import (normalize_to_list_of_substitutions,
                                  perform_substitutions)
    from launch_ros.actions import Node as LNode
    from boids_swarm.launch_util import parse_obstacles
    from boids_swarm.world_gen import WorldGenerator
    ctx, actions = _launch_actions(env, seed)
    expected = WorldGenerator(seed, 20.0).generate(env).obstacles_flat()
    nodes = {a.node_executable: a for a in actions if isinstance(a, LNode)}
    inc = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(inc) == 1, 'evader:=nav2 must include nav2_target.launch.py'
    args = {perform_substitutions(ctx, normalize_to_list_of_substitutions(k)):
            perform_substitutions(ctx, normalize_to_list_of_substitutions(v))
            for k, v in inc[0].launch_arguments}
    assert [float(v) for v in _param(nodes['pygame_sim'], 'obstacles')] == expected
    assert [float(v) for v in _param(nodes['target_controller'],
                                     'obstacles')] == expected
    assert parse_obstacles(args['obstacles']) == expected      # -> bridge
    assert args['cmd_topic'] == '/target/nav2_cmd_vel'
    assert float(args['world_size']) == 20.0


def test_reactive_evader_does_not_start_nav2():
    from launch.actions import IncludeLaunchDescription
    mod = _load_launch('pursuit.launch.py')
    ctx2, _ = _launch_actions('open', 7)
    ctx2.launch_configurations['evader'] = 'reactive'
    acts2 = mod.launch_setup(ctx2)
    assert not any(isinstance(a, IncludeLaunchDescription) for a in acts2)


def test_controller_limits_come_from_params_not_the_yaml():
    mod = _load_launch('nav2_target.launch.py')
    cfg = os.path.join(PKG, 'config', 'nav2_target.yaml')
    ov = mod.controller_overrides(cfg, 3.6, 1.2)          # shipped limits
    assert ov['FollowPath.desired_linear_vel'] == pytest.approx(3.6)
    assert ov['FollowPath.regulated_linear_scaling_min_radius'] == pytest.approx(3.0)
    ov2 = mod.controller_overrides(cfg, 4.0, 2.0)         # a different balance
    assert ov2['FollowPath.desired_linear_vel'] == pytest.approx(4.0)
    assert ov2['FollowPath.regulated_linear_scaling_min_radius'] == pytest.approx(2.0)
    mppi = mod.controller_overrides(
        os.path.join(PKG, 'config', 'nav2_target_mppi.yaml'), 3.6, 1.2)
    assert mppi == {'FollowPath.vx_max': 3.6, 'FollowPath.wz_max': 1.2}
