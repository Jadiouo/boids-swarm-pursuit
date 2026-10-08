"""Control-panel mode switching against the real launch (no mocks).

Starts `pursuit.launch.py ui:=true headless:=false` under SDL's dummy video
driver (a real window-owning sim, nobody looking at it), switches mode
through `/ui/mode_request` (the same path a mode-button click takes) and
checks, from the outside, that:

  * the controllers really restart with the new perception / sharing_mode
    (queried from the live nodes' parameters),
  * the old stack's whole process group is gone (no orphans) and exactly one
    stack is alive after the switch,
  * a bad request leaves the running stack alone,
  * closing the sim (SIGINT to the launch, or even SIGKILL to the sim)
    leaves no process behind.

Run explicitly after `colcon build`:
    cd ros2_ws && source install/setup.bash
    SDL_VIDEODRIVER=dummy ROS_DOMAIN_ID=<free id> \\
        python3 -m pytest src/boids_swarm/ros_test/test_ui_modes_ros_integration.py

The Nav2 mode (6-8 s warm-up, four more processes) is covered by
test_switch_to_nav2_mode_and_back, which is slow and marked accordingly.
"""

import json
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

N = 3
START_TIMEOUT = 120.0


def group_pids(pgid):
    out = subprocess.run(['ps', '-o', 'pid=', '-g', str(pgid)],
                         capture_output=True, text=True).stdout
    return [int(x) for x in out.split()]


def group_alive(pgid):
    return bool(group_pids(pgid))


def children_of(pid):
    out = subprocess.run(['ps', '-o', 'pid=,args=', '--ppid', str(pid)],
                         capture_output=True, text=True).stdout
    return [(int(ln.split(None, 1)[0]), ln.split(None, 1)[1])
            for ln in out.splitlines() if ln.strip()]


def wait_until(pred, timeout, what, step=0.2):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        v = pred()
        if v:
            return v
        time.sleep(step)
    raise AssertionError(f'timed out after {timeout:.0f}s: {what}')


class Probe(Node):
    def __init__(self):
        super().__init__('ui_modes_test_probe')
        self.status = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/ui/stack_status', self._on, qos)
        self.req = self.create_publisher(String, '/ui/mode_request', 10)
        self._pclients = {}

    def _on(self, msg):
        self.status = json.loads(msg.data)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_param(self, node, name, timeout=10.0):
        cli = self._pclients.get(node)
        if cli is None:
            cli = self._pclients[node] = self.create_client(
                GetParameters, f'{node}/get_parameters')
        assert cli.wait_for_service(timeout_sec=timeout), node
        req = GetParameters.Request()
        req.names = [name]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        v = fut.result().values[0]
        return v.string_value if v.type == 4 else \
            (v.integer_value if v.type == 2 else v.double_value)

    def request(self, text):
        self.req.publish(String(data=text))


@pytest.fixture
def world():
    yield from _world()


def _world(preexec_fn=None):
    if 'ROS_DOMAIN_ID' not in os.environ:
        pytest.skip('set ROS_DOMAIN_ID to a free domain first')
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy')
    rclpy.init()
    probe = Probe()
    # never share a domain with somebody's running game
    probe.spin_for(1.5)
    if '/pygame_sim' in [f'{ns.rstrip("/")}/{n}' for n, ns in
                         probe.get_node_names_and_namespaces()]:
        probe.destroy_node()
        rclpy.shutdown()
        pytest.skip('a pygame_sim is already running in this ROS domain')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'boids_swarm', 'pursuit.launch.py',
         f'num_agents:={N}', 'ui:=true', 'headless:=false', 'seed:=3'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        start_new_session=True, preexec_fn=preexec_fn)
    stacks = set()
    ctx = {'probe': probe, 'launch': launch, 'stacks': stacks}
    try:
        yield ctx
    finally:
        for pgid in list(stacks):
            if group_alive(pgid):
                os.killpg(pgid, signal.SIGKILL)
        try:
            os.killpg(launch.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        launch.wait(timeout=10)
        probe.destroy_node()
        rclpy.shutdown()


def running(ctx, generation=None, timeout=START_TIMEOUT):
    probe = ctx['probe']

    def ok():
        probe.spin_for(0.2)
        st = probe.status
        if st and st['state'] == 'failed':
            raise AssertionError(f"stack failed: {st['detail']}")
        if st and st['state'] == 'running' and (
                generation is None or st['generation'] == generation):
            ctx['stacks'].add(st['pid'])
            return st
        return None
    return wait_until(ok, timeout, f'stack running (gen {generation})')


def test_switch_baseline_to_sensor_relay_restarts_controllers(world):
    probe = world['probe']
    st1 = running(world, 1)
    assert st1['mode'] == 'baseline'
    for i in range(N):
        assert probe.get_param(f'/agent{i}/boid_controller',
                               'perception_mode') == 'perfect'
    pg1 = st1['pid']
    assert group_alive(pg1)
    first_pids = set(group_pids(pg1))
    assert len(first_pids) >= N + 2             # launch + target + boids

    probe.request('sensor_ros')
    st2 = running(world, 2)
    pg2 = st2['pid']
    assert st2['mode'] == 'sensor_ros' and pg2 != pg1
    # old stack: whole process group gone, new one alive
    assert not group_alive(pg1)
    assert not (first_pids & set(group_pids(pg2)))
    for i in range(N):
        node = f'/agent{i}/boid_controller'
        assert probe.get_param(node, 'perception_mode') == 'sensor'
        assert probe.get_param(node, 'sharing_mode') == 'ros'
    assert probe.get_param('/pygame_sim', 'perception_mode') == 'sensor'
    assert probe.get_param('/pygame_sim', 'sharing_mode') == 'ros'
    # exactly one live stack, and no controller is duplicated in the graph
    time.sleep(1.0)
    probe.spin_for(0.5)
    names = [f'{ns.rstrip("/")}/{n}' for n, ns in
             probe.get_node_names_and_namespaces()]
    assert sorted(x for x in names if x.endswith('boid_controller')) == \
        [f'/agent{i}/boid_controller' for i in range(N)]


def test_bad_request_keeps_the_running_stack(world):
    probe = world['probe']
    st = running(world, 1)
    probe.request('warp_drive')
    probe.request('{"overrides": {"perception": "perfect", '
                  '"sharing_mode": "ros"}}')          # ros needs sensor
    probe.spin_for(2.0)
    now = probe.status
    assert now['generation'] == 1 and now['state'] == 'running'
    assert group_alive(st['pid'])


def test_scene_change_rebuilds_the_world_under_the_open_window(world):
    probe = world['probe']
    running(world, 1)
    probe.request('{"overrides": {"num_agents": 4, "seed": 5}}')
    st = running(world, 2)
    assert st['applied']['num_agents'] == 4
    assert probe.get_param('/pygame_sim', 'num_agents') == 4
    assert probe.get_param('/agent3/boid_controller', 'num_agents') == 4
    probe.spin_for(1.0)
    names = [f'{ns.rstrip("/")}/{n}' for n, ns in
             probe.get_node_names_and_namespaces()]
    assert '/agent3/boid_controller' in names


def test_closing_the_launch_leaves_no_orphans(world):
    st = running(world, 1)
    pg = st['pid']
    launch = world['launch']
    launch.send_signal(signal.SIGINT)               # what Ctrl-C does
    launch.wait(timeout=60)
    wait_until(lambda: not group_alive(pg), 30, 'stack group gone')
    assert children_of(launch.pid) == []


def descendants(pid):
    out = subprocess.run(['ps', '-e', '-o', 'pid=,ppid='],
                         capture_output=True, text=True).stdout.split()
    parent = dict(zip(map(int, out[::2]), map(int, out[1::2])))
    found, grew = {pid}, True
    while grew:
        grew = False
        for p, pp in parent.items():
            if pp in found and p not in found:
                found.add(p)
                grew = True
    return found - {pid}


def live(pids):
    def alive(p):
        try:
            with open(f'/proc/{p}/stat') as f:
                return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
        except OSError:
            return False
    return [p for p in pids if alive(p)]


SIGINT_DEADLINE = 6.0     # seconds from `kill -INT launch` to nothing left


def _sigint_and_time(world):
    st = running(world, 1)
    launch = world['launch']
    tree = descendants(launch.pid)
    assert len(tree) >= 1 + N                      # sim + controllers
    t0 = time.monotonic()
    launch.send_signal(signal.SIGINT)
    wait_until(lambda: launch.poll() is not None and not live(tree),
               30, 'launch and everything under it gone')
    took = time.monotonic() - t0
    assert not group_alive(st['pid'])
    return took


def test_sigint_to_the_launch_ends_everything_quickly(world):
    assert _sigint_and_time(world) < SIGINT_DEADLINE


@pytest.fixture
def world_sigint_ignored():
    """Same world, but the launch starts with SIGINT ignored, as a
    background job of a non-interactive shell (or nohup) does. Before the
    launch_util fix, `kill -INT` then did nothing at all."""
    yield from _world(
        lambda: signal.signal(signal.SIGINT, signal.SIG_IGN))


def test_sigint_works_even_when_it_was_inherited_as_ignored(
        world_sigint_ignored):
    assert _sigint_and_time(world_sigint_ignored) < SIGINT_DEADLINE


def test_sim_killed_with_sigkill_still_takes_the_stack_down(world):
    st = running(world, 1)
    pg = st['pid']
    sim = next(p for p, cmd in children_of(world['launch'].pid)
               if 'pygame_sim' in cmd)
    os.kill(sim, signal.SIGKILL)
    # the stack's parent watchdog notices within ~1 s and shuts it down
    wait_until(lambda: not group_alive(pg), 45, 'stack group gone')


@pytest.mark.slow
def test_switch_to_nav2_mode_and_back(world):
    probe = world['probe']
    running(world, 1)
    probe.request('nav2')
    st = running(world, 2, timeout=240)
    assert st['mode'] == 'nav2'
    names = [f'{ns.rstrip("/")}/{n}' for n, ns in
             probe.get_node_names_and_namespaces()]
    for want in ('/planner_server', '/controller_server', '/nav2_bridge'):
        assert want in names, want
    assert probe.get_param('/target_controller', 'evader') == 'nav2'
    probe.request('baseline')
    st3 = running(world, 3)
    assert not group_alive(st['pid'])
    probe.spin_for(2.0)
    names = [f'{ns.rstrip("/")}/{n}' for n, ns in
             probe.get_node_names_and_namespaces()]
    assert '/planner_server' not in names
    assert st3['mode'] == 'baseline'
