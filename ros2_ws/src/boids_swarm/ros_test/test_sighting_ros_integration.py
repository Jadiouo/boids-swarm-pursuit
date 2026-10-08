"""Real two-controller ROS path checks; run explicitly after colcon build."""

import json
import os
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from turtlesim.msg import Pose
from boids_swarm_msgs.msg import EpisodeState, TargetSighting


class WireFixture(Node):
    def __init__(self):
        super().__init__('sighting_ros_test_fixture')
        self.events = []
        self.status = {f'agent{i}': [] for i in range(3)}
        self.commands = {f'agent{i}': [] for i in range(3)}
        self.shared = []
        epoch_qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        shared_qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.epoch_pub = self.create_publisher(
            EpisodeState, '/simulation/episode_state', epoch_qos)
        self.shared_pub = self.create_publisher(
            TargetSighting, '/swarm/target_sightings', shared_qos)
        self.local_pubs = [self.create_publisher(
            TargetSighting, f'/agent{i}/local_target_sighting', 10)
            for i in range(3)]
        self.pose_pubs = [self.create_publisher(Pose, f'/agent{i}/pose', 10)
                          for i in range(3)]
        self.detection_pubs = [self.create_publisher(
            Float32MultiArray, f'/agent{i}/detections', 10)
            for i in range(3)]
        self.create_subscription(String, '/swarm/relay_events',
            lambda m: self.events.append(json.loads(m.data)), 100)
        self.create_subscription(TargetSighting, '/swarm/target_sightings',
            self.shared.append, shared_qos)
        for i in range(3):
            self.create_subscription(String, f'/agent{i}/target_track_status',
                lambda m, k=i: self.status[f'agent{k}'].append(
                    json.loads(m.data)), 100)
            self.create_subscription(Twist, f'/agent{i}/cmd_vel',
                lambda m, k=i: self.commands[f'agent{k}'].append(
                    (m.linear.x, m.angular.z)), 100)

    def publish_epoch(self, episode):
        msg = EpisodeState()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.episode_id = episode
        self.epoch_pub.publish(msg)

    def publish_world(self):
        for i in range(3):
            pose = Pose()
            pose.x, pose.y, pose.theta = 5.0 + 2.0*i, 5.0, 0.0
            self.pose_pubs[i].publish(pose)
            peer = (i + 1) % 3
            det = Float32MultiArray()
            det.data = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, float(peer)]
            self.detection_pubs[i].publish(det)

    def make_sighting(self, episode, sequence, sender='agent0', stamp=None):
        msg = TargetSighting()
        msg.header.frame_id = 'world'
        msg.header.stamp = stamp or self.get_clock().now().to_msg()
        msg.sender_id, msg.episode_id = sender, episode
        msg.sequence, msg.target_id = sequence, 'target'
        msg.sender_position.x = 5.0
        msg.sender_position.y = 5.0
        msg.target_position.x = 8.0 + sequence * 0.2
        msg.target_position.y = 5.0
        msg.covariance_xy = [0.04, 0.0, 0.0, 0.04]
        msg.confidence, msg.valid_for_sec = 0.9, 0.5
        return msg

    def drive(self, seconds, *, publish_world=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish_world:
                self.publish_world()
            # Drain the fixture's input queues faster than controller status
            # and pose/detection publication; a single spin per 50ms can
            # otherwise make assertions observe stale history.
            for _ in range(12):
                rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.015)


def _start_controller(namespace, count, extra=()):
    args = ['ros2', 'run', 'boids_swarm', 'boid_controller', '--ros-args',
        '-r', f'__ns:=/{namespace}', '-p', f'num_agents:={count}',
        '-p', 'perception_mode:=sensor', '-p', 'sharing_mode:=ros',
        '-p', 'use_sim_time:=false', '-p', 'track_max_age:=0.15',
        '-p', 'sighting_timeout:=0.5', '-p', 'radio_range:=8.0', *extra]
    return subprocess.Popen(args, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


def _stop(processes):
    """Kill each `ros2 run` process GROUP; terminate() alone orphans the
    controller child (it keeps running and pollutes later runs)."""
    import signal
    for process in processes:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    for process in processes:
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=2)


# Every _wait_for is a positive "poll until the condition holds" wait, so a
# longer timeout cannot change what is asserted; it only absorbs scheduling
# delay (process start-up, DDS discovery) on a loaded machine. The multiplier
# can be overridden through the environment.
_WAIT_SCALE = float(os.environ.get('SIGHTING_TEST_WAIT_SCALE', '5'))


def _wait_for(fixture, predicate, seconds=3.0):
    deadline = time.monotonic() + seconds * _WAIT_SCALE
    while time.monotonic() < deadline:
        fixture.drive(0.05)
        if predicate():
            return True
    return False


def test_controller_to_controller_relay_ttl_reset_old_epoch_and_late_join(monkeypatch):
    domain = int(os.environ.get('ROS_DOMAIN_ID', 180 + os.getpid() % 45))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    if not rclpy.ok():
        rclpy.init(args=None)
    fixture = WireFixture()
    controllers = []
    try:
        controllers = [_start_controller('agent0', 3),
                       _start_controller('agent1', 3)]
        assert _wait_for(fixture,
            lambda: fixture.epoch_pub.get_subscription_count() >= 2, 6.0)
        fixture.publish_epoch(1)
        fixture.drive(0.4)
        status_b = fixture.status['agent1']
        before_relay = len(status_b)
        direct = fixture.make_sighting(1, 1)
        fixture.local_pubs[0].publish(direct)
        assert _wait_for(fixture, lambda: any(
            s.get('episode_id') == 1 and s.get('mode') == 'relay'
            and s.get('belief_valid') for s in status_b[before_relay:]
            if s.get('event') == 'cycle'), 2.0), \
            f'no first relay status: {status_b[-10:]} events={fixture.events[-10:]}'
        first_belief = next(s for s in status_b[before_relay:]
            if s.get('mode') == 'relay' and s.get('belief_valid')
            and s.get('event') == 'cycle')
        source_stamp = first_belief['observation_stamp']
        assert any(_same_payload(direct, m) for m in fixture.shared)
        assert any(e.get('receiver_id') == 'agent1' and e.get('accepted')
                   for e in fixture.events)
        assert not any(m.sender_id == 'agent1' for m in fixture.shared)

        # Keep neighbor detections flowing while the target publisher is silent.
        assert _wait_for(fixture, lambda: any(
            s.get('episode_id') == 1 and s.get('mode') == 'search'
            and not s.get('belief_valid') and s.get('stamp', 0) > source_stamp + 0.5
            for s in status_b if s.get('event') == 'cycle'), 1.5)
        initial_count = len(status_b)
        relay_record = fixture.make_sighting(1, 2)
        direct_record = fixture.make_sighting(1, 1, sender='agent1')
        direct_record.target_position.x = 10.0
        direct_stamp = (direct_record.header.stamp.sec +
                        direct_record.header.stamp.nanosec * 1e-9)
        fixture.shared_pub.publish(relay_record)
        fixture.local_pubs[1].publish(direct_record)
        assert _wait_for(fixture, lambda: any(
            s.get('mode') == 'direct' and s.get('last_sender_id') == 'agent1'
            and s.get('last_sequence') == 1
            and s.get('observation_stamp') == pytest.approx(direct_stamp)
            for s in status_b[initial_count:] if s.get('event') == 'cycle'), 1.0), \
            f'no direct winner: statuses={status_b[-8:]} events={fixture.events[-12:]}'

        # A duplicate callback cannot renew the original observation stamp.
        direct2 = fixture.make_sighting(1, 2, sender='agent1')
        direct2.target_position.x = 10.5
        direct2.target_position.y = 5.0
        fixture.local_pubs[1].publish(direct2)
        assert _wait_for(fixture, lambda: any(
            s.get('mode') == 'direct' and s.get('last_sequence') == 2
            and s.get('belief_valid') for s in status_b
            if s.get('event') == 'cycle'), 1.0)
        direct_cmd = fixture.commands['agent1'][-1]
        direct2_stamp = direct2.header.stamp.sec + direct2.header.stamp.nanosec*1e-9
        fixture.drive(0.2)
        fixture.local_pubs[1].publish(direct2)
        assert _wait_for(fixture, lambda: any(
            s.get('mode') == 'search' and not s.get('belief_valid')
            and s.get('stamp', 0) > direct2_stamp + 0.5
            for s in status_b if s.get('event') == 'cycle'), 1.5)
        search_cmd = fixture.commands['agent1'][-1]
        assert abs(search_cmd[0]-direct_cmd[0]) + \
               abs(search_cmd[1]-direct_cmd[1]) > 0.1

        # Reacquire before reset; the reset itself arrives with no sightings.
        before = len(status_b)
        fixture.local_pubs[0].publish(fixture.make_sighting(1, 3))
        assert _wait_for(fixture, lambda: any(
            s.get('mode') == 'relay' and s.get('belief_valid')
            for s in status_b[before:] if s.get('event') == 'cycle'), 1.0)
        before = len(status_b)
        fixture.publish_epoch(2)
        assert _wait_for(fixture, lambda: any(
            s.get('episode_id') == 2 and s.get('event') == 'transition'
            and s.get('reason') == 'episode_reset'
            and not s.get('belief_valid') and s.get('mode') == 'search'
            for s in status_b[before:]), 1.0)

        old = fixture.make_sighting(1, 3)
        fixture.shared_pub.publish(old)
        assert _wait_for(fixture, lambda: any(
            e.get('reason') == 'old_episode' for e in fixture.events), 1.0)
        future = fixture.make_sighting(3, 1)
        fixture.shared_pub.publish(future)
        assert _wait_for(fixture, lambda: any(
            e.get('reason') == 'future_episode'
            for e in fixture.events), 1.0)

        late = _start_controller('agent2', 3)
        controllers.append(late)
        assert _wait_for(fixture, lambda: any(
            s.get('episode_id') == 2 and s.get('reason') == 'episode_reset'
            and s.get('event') == 'transition'
            for s in fixture.status['agent2']), 2.0)
    finally:
        _stop(controllers)
        fixture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _same_payload(a, b):
    return (a.episode_id == b.episode_id and a.sequence == b.sequence
            and a.sender_id == b.sender_id
            and a.header.stamp == b.header.stamp
            and a.target_position.x == b.target_position.x
            and a.target_position.y == b.target_position.y
            and tuple(a.covariance_xy) == tuple(b.covariance_xy))


def test_sensor_covariance_uses_resolved_sensor_range():
    from boids_swarm.perception import target_measurement_world
    sample = (4.0, 0.4, 0.0, 0.0, 1.0, 2.0)
    short = target_measurement_world((1.0, 2.0, 0.3), sample,
        stamp=1.0, sequence=1, episode_id=1, sender_id='agent0',
        range_sigma=0.02, bearing_sigma=0.03, sensor_range=4.0)
    long = target_measurement_world((1.0, 2.0, 0.3), sample,
        stamp=1.0, sequence=1, episode_id=1, sender_id='agent0',
        range_sigma=0.02, bearing_sigma=0.03, sensor_range=12.0)
    assert tuple(short.covariance_xy) != tuple(long.covariance_xy)


def test_qos_depth_parameter_and_status_counters(monkeypatch):
    """R-01 depth is applied to the live endpoints; R-03 counters are
    reported by the receiver itself in its (reliable) track status."""
    domain = int(os.environ.get('ROS_DOMAIN_ID', 180 + os.getpid() % 45))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    if not rclpy.ok():
        rclpy.init(args=None)
    fixture = WireFixture()
    controllers = []
    try:
        controllers = [
            _start_controller('agent0', 3,
                ('-p', 'shared_sighting_qos_depth:=10')),
            _start_controller('agent1', 3,
                ('-p', 'shared_sighting_qos_depth:=10'))]
        assert _wait_for(fixture,
            lambda: fixture.epoch_pub.get_subscription_count() >= 2, 6.0)
        fixture.publish_epoch(1)
        fixture.drive(0.4)
        # sender agent0 at (5,5); agent1 pose is (7,5) -> in range
        fixture.local_pubs[0].publish(fixture.make_sighting(1, 1))
        # a sighting from a far-away sender position -> out of range for agent1
        far = fixture.make_sighting(1, 2, sender='agent2')
        far.sender_position.x, far.sender_position.y = 40.0, 40.0
        fixture.shared_pub.publish(far)

        def last(agent):
            rows = [s for s in fixture.status[agent]
                    if s.get('event') == 'cycle']
            return rows[-1] if rows else {}
        assert _wait_for(fixture, lambda:
            last('agent1').get('sightings_received_in_range', 0) >= 1
            and last('agent1').get('sightings_received_out_of_range', 0) >= 1
            and last('agent0').get('sightings_received_self', 0) >= 1, 3.0), \
            (last('agent0'), last('agent1'))
        a0, a1 = last('agent0'), last('agent1')
        assert a0['shared_sighting_qos_depth'] == 10
        assert a1['shared_sighting_qos_depth'] == 10
        assert a0['sightings_published'] >= 1           # agent0 relayed once
        assert a1['sightings_published'] == 0
        assert a0['sightings_received_self'] >= 1
        for a in (a0, a1):
            assert a['sightings_received_total'] == (
                a['sightings_received_in_range']
                + a['sightings_received_out_of_range']
                + a['sightings_received_self']
                + a['sightings_received_unclassified'])
    finally:
        _stop(controllers)
        fixture.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
