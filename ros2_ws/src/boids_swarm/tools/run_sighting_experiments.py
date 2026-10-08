#!/usr/bin/env python3
"""Run the bounded CPU-only one-hop comparison and save public ROS evidence."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray
from boids_swarm_msgs.msg import TargetSighting, EpisodeState


ROOT = Path(__file__).resolve().parents[4]
MODES = ('off', 'oracle', 'ros')
SEEDS = (11, 23, 37)
STALE_REASONS = {'stale', 'old_episode', 'future_episode', 'future_stamp'}


def fingerprint_files():
    roots = [ROOT / 'ros2_ws/src/boids_swarm',
             ROOT / 'ros2_ws/src/boids_swarm_msgs']
    files = []
    for base in roots:
        for path in base.rglob('*'):
            if not path.is_file() or any(part in {
                    '__pycache__', 'build', 'install', 'log'}
                    for part in path.parts):
                continue
            if path.suffix in {'.py', '.msg', '.yaml', '.xml', '.txt'}:
                files.append(path)
    files.extend([ROOT / 'README.md'])
    return sorted(set(files))


def source_fingerprint():
    files = fingerprint_files()
    digest = hashlib.sha256()
    for path in sorted(set(files)):
        digest.update(path.relative_to(ROOT).as_posix().encode())
        digest.update(b'\0')
        digest.update(path.read_bytes())
        digest.update(b'\0')
    return digest.hexdigest()


class Capture(Node):
    def __init__(self, num_agents):
        super().__init__('sighting_experiment_capture')
        self.observations = []
        self.relay_events = []
        self.statuses = []
        self.local_sightings = []
        self.shared_sightings = []
        self.episode_states = []
        self.target_array_entries = 0
        self.create_subscription(String, '/swarm/observations',
            lambda m: self.observations.append(json.loads(m.data)), 100)
        self.create_subscription(String, '/swarm/relay_events',
            lambda m: self.relay_events.append(json.loads(m.data)), 100)
        shared_qos = QoSProfile(depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(TargetSighting, '/swarm/target_sightings',
            self.shared_sightings.append, shared_qos)
        self.create_subscription(EpisodeState, '/simulation/episode_state',
            self.episode_states.append, 1)
        for i in range(num_agents):
            self.create_subscription(String, f'/agent{i}/target_track_status',
                lambda m: self.statuses.append(json.loads(m.data)), 100)
            self.create_subscription(TargetSighting,
                f'/agent{i}/local_target_sighting',
                lambda m: self.local_sightings.append(m), 100)
            self.create_subscription(Float32MultiArray,
                f'/agent{i}/detections', self._count_target_detections, 100)

    def _count_target_detections(self, msg):
        data = list(msg.data)
        if not data:
            return
        count = int(data[0])
        for k in range(count):
            base = 1 + 6*k
            if base + 5 < len(data) and data[base+4] >= 0.5:
                self.target_array_entries += 1


def _sighting_key(msg):
    return (int(msg.episode_id), msg.sender_id, int(msg.sequence))


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def sighting_row(msg):
    return {
        'episode_id': int(msg.episode_id), 'sequence': int(msg.sequence),
        'sender_id': msg.sender_id, 'target_id': msg.target_id,
        'stamp_sec': int(msg.header.stamp.sec),
        'stamp_nanosec': int(msg.header.stamp.nanosec),
        'frame_id': msg.header.frame_id,
        'sender_position': [msg.sender_position.x, msg.sender_position.y,
                            msg.sender_position.z],
        'target_position': [msg.target_position.x, msg.target_position.y,
                            msg.target_position.z],
        'covariance_xy': list(msg.covariance_xy),
        'confidence': float(msg.confidence),
        'valid_for_sec': float(msg.valid_for_sec),
    }


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int((len(ordered) - 1) * fraction + 0.5))
    return ordered[index]


def metrics(mode, collector):
    cycle_status = [s for s in collector.statuses
                    if s.get('event') == 'cycle'
                    and s.get('own_pose_fresh')]
    valid = sum(bool(s.get('belief_valid')) for s in cycle_status)
    accepted = [e for e in collector.relay_events if e.get('accepted')]
    ages = [max(0.0, float(e['receive_stamp']) -
                float(e['observation_stamp'])) for e in accepted]
    stale = sum(e.get('reason') in STALE_REASONS
                for e in collector.relay_events)
    if mode == 'ros':
        opportunities = {e['receiver_id'] for e in collector.relay_events
                         if e.get('opportunity')}
        recipients = {e['receiver_id'] for e in accepted}
        coverage = (len(recipients & opportunities) / len(opportunities)
                    if opportunities else None)
        coverage_denominator = len(opportunities)
    else:
        coverage = None
        coverage_denominator = None
    observations = {_sighting_key(m): m for m in collector.local_sightings}
    shared = {_sighting_key(m): m for m in collector.shared_sightings}
    matched = set(observations) & set(shared)
    unchanged = 0
    for key in matched:
        a, b = observations[key], shared[key]
        same = (a.header.frame_id == b.header.frame_id and
                a.target_id == b.target_id and
                _stamp(a) == _stamp(b) and
                a.target_position.x == b.target_position.x and
                a.target_position.y == b.target_position.y and
                a.target_position.z == b.target_position.z and
                a.sender_position.x == b.sender_position.x and
                a.sender_position.y == b.sender_position.y and
                a.sender_position.z == b.sender_position.z and
                tuple(a.covariance_xy) == tuple(b.covariance_xy) and
                a.confidence == b.confidence and
                a.valid_for_sec == b.valid_for_sec)
        unchanged += int(same)
    return {
        'wire_relay_applicable': mode == 'ros',
        'simulation_time_observation_age_sec_mean':
            (sum(ages)/len(ages) if ages else None),
        'simulation_time_observation_age_zero_fraction':
            (sum(age == 0.0 for age in ages)/len(ages) if ages else None),
        'simulation_time_observation_age_sec_p95': _percentile(ages, 0.95),
        'simulation_time_observation_age_sec_max': max(ages) if ages else None,
        'age_clock_note': 'ROS simulation time; not wall-clock DDS latency',
        'accepted_relay_count': len(accepted),
        'receiver_opportunity_coverage': coverage,
        'receiver_opportunity_coverage_denominator_distinct_receivers':
            coverage_denominator,
        'receiver_opportunity_coverage_definition':
            'distinct receivers accepted at least once / distinct receivers with at least one fresh in-range opportunity; not packet delivery ratio',
        'track_valid_fraction': (valid/len(cycle_status)
                                 if cycle_status else None),
        'track_valid_numerator': valid,
        'track_valid_denominator': len(cycle_status),
        'stale_rejection_count': stale,
        'observation_local_wire_keys': len(matched),
        'matched_local_wire_keys': [list(key) for key in sorted(matched)],
        'unchanged_shared_payload_count': unchanged,
        'ros_mode_target_entries_in_legacy_array':
            collector.target_array_entries,
    }


def run_case(mode, seed, output, repeat, timeout_sec=30.0):
    output.mkdir(parents=True, exist_ok=False)
    domain = 180 + (os.getpid() + seed + repeat * 31) % 50
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID=str(domain), SDL_VIDEODRIVER='dummy',
               SDL_AUDIODRIVER='dummy')
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    os.environ['SDL_AUDIODRIVER'] = 'dummy'
    # pygame must already be importable (apt python3-pygame or pip --user);
    # PYTHONPATH is inherited from the caller's environment.
    command = ['ros2', 'launch', 'boids_swarm', 'pursuit.launch.py',
        'headless:=true', 'ui:=false', 'num_agents:=4',
        'episodes_max:=1', f'time_limit:={timeout_sec}', f'seed:={seed}',
        'strategy:=intercept', 'game_mode:=ai', 'perception:=sensor',
        'capture_mode:=hull', 'env:=open', f'sharing_mode:={mode}']
    rclpy.init(args=None)
    capture = Capture(4)
    proc = subprocess.Popen(command, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env,
                            cwd=ROOT / 'ros2_ws')
    try:
        while proc.poll() is None:
            rclpy.spin_once(capture, timeout_sec=0.05)
        for _ in range(8):
            rclpy.spin_once(capture, timeout_sec=0.05)
        log, _ = proc.communicate(timeout=5)
    finally:
        capture.destroy_node()
        rclpy.shutdown()
    (output / 'launch.log').write_text(log)
    (output / 'observations.jsonl').write_text(''.join(
        json.dumps(row, sort_keys=True) + '\n'
        for row in capture.observations))
    (output / 'relay_events.jsonl').write_text(''.join(
        json.dumps(row, sort_keys=True) + '\n'
        for row in capture.relay_events))
    (output / 'track_status.jsonl').write_text(''.join(
        json.dumps(row, sort_keys=True) + '\n'
        for row in capture.statuses))
    for filename, messages in (('local_sightings.jsonl', capture.local_sightings),
                                ('shared_sightings.jsonl', capture.shared_sightings)):
        (output / filename).write_text(''.join(
            json.dumps(sighting_row(msg), sort_keys=True) + '\n'
            for msg in messages))
    result_match = re.search(r'EPISODE 1 result=(\w+) t=([0-9.]+)s', log)
    commit = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                            text=True, capture_output=True, check=False).stdout.strip()
    dirty = subprocess.run(['git', 'status', '--short'], cwd=ROOT,
                           text=True, capture_output=True).stdout.strip()
    params_path = ROOT / 'ros2_ws/src/boids_swarm/config/params.yaml'
    try:
        import yaml
        with params_path.open() as f:
            defaults = yaml.safe_load(f)['/**']['ros__parameters']
    except Exception:
        defaults = {}
    metrics_result = metrics(mode, capture)
    config = {
        'mode': mode, 'seed': seed, 'repeat': repeat,
        'parameters': {**defaults, 'num_agents': 4, 'seed': seed,
            'strategy': 'intercept', 'game_mode': 'ai', 'env': 'open',
            'perception_mode': 'sensor', 'sharing_mode': mode,
            'capture_mode': 'hull', 'episodes_max': 1,
            'episode_time_limit': timeout_sec, 'headless': True, 'ui': False},
        'ros_domain_id': domain, 'ros_version': os.environ.get('ROS_DISTRO', 'jazzy'),
        'git_commit': commit, 'working_tree_dirty': bool(dirty),
        'source_fingerprint_sha256': source_fingerprint(),
        'exit_code': proc.returncode,
        'run_status': 'completed' if proc.returncode == 0 and result_match else 'failed',
        'capture_result': result_match.group(1) if result_match else None,
        'capture_time_sec': float(result_match.group(2)) if result_match else None,
        'episode_state_count': len(capture.episode_states),
        'metrics': metrics_result,
    }
    (output / 'run_config.json').write_text(json.dumps(config, indent=2, sort_keys=True))
    (output / 'summary.json').write_text(json.dumps(
        {'mode': mode, 'seed': seed, 'repeat': repeat,
         'capture_result': config['capture_result'],
         'capture_time_sec': config['capture_time_sec'],
         'metrics': metrics_result}, indent=2, sort_keys=True))
    if config['run_status'] != 'completed':
        raise RuntimeError(f'run failed; inspect {output / "launch.log"}')
    if mode == 'ros' and metrics_result['unchanged_shared_payload_count'] == 0:
        raise RuntimeError(f'no unchanged local-to-shared wire payloads in {output}')
    if mode == 'ros' and metrics_result['ros_mode_target_entries_in_legacy_array']:
        raise RuntimeError(f'ROS mode leaked target to legacy array in {output}')
    return config


def write_source_snapshot(output):
    import tarfile
    archive = output / 'source_snapshot.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for path in fingerprint_files():
            tar.add(path, arcname=path.relative_to(ROOT).as_posix())
    return archive


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/sighting-relay')
    parser.add_argument('--timeout-sec', type=float, default=30.0)
    parser.add_argument('--smoke', action='store_true',
                        help='run one short ROS-mode path check only')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    runs = []
    if not args.smoke:
        write_source_snapshot(args.output)
    matrix = [('ros', 11, 0)] if args.smoke else [
        (mode, seed, 0) for mode in MODES for seed in SEEDS]
    if not args.smoke:
        matrix += [(mode, 11, 1) for mode in MODES]
    for mode, seed, repeat in matrix:
        label = f'{mode}-seed{seed}-' + ('repeat' if repeat else 'base')
        run = run_case(mode, seed, args.output / label, repeat,
                       timeout_sec=4.0 if args.smoke else args.timeout_sec)
        runs.append(run)
        print(f"{label}: {run['run_status']} metrics={run['metrics']}", flush=True)
    aggregate = {'configured_run_count': len(runs), 'runs': runs,
        'by_mode': {mode: [r for r in runs if r['mode'] == mode]
                    for mode in sorted({r['mode'] for r in runs})}}
    (args.output / 'summary.json').write_text(json.dumps(aggregate, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
