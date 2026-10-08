#!/usr/bin/env python3
"""Phase-2 diagnostic: one run per invocation cell, raw JSONL + wall/CPU cost.

Reuses Capture/metrics from run_sighting_experiments.py; adds a deep
(depth=300) BEST_EFFORT shared-sighting collector so collector loss can be
separated from receiver loss, and launches through diag_pursuit.launch.py to
override comm_range/radio_range without touching params.yaml.
usage: run_sighting_diagnostic.py --out DIR --cells ros:4:11,oracle:12:23 --domain 233
"""
import argparse, json, os, re, resource, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from boids_swarm_msgs.msg import TargetSighting
import run_sighting_experiments as base

ROOT = base.ROOT
LAUNCH = Path(__file__).resolve().parent / 'diag_pursuit.launch.py'


class DeepCapture(Node):
    """Slim collector (no /detections subscriptions, to keep the single Python
    collector from becoming the bottleneck). Three views of the shared topic:
    depth1 (original collector QoS), depth300 (best available 'published' view)."""
    def __init__(self, n):
        super().__init__('sighting_diag_capture')
        self.observations, self.relay_events, self.statuses = [], [], []
        self.local_sightings, self.shared_sightings, self.shared_deep = [], [], []
        self.target_array_entries = None      # not collected in this runner
        self.create_subscription(String, '/swarm/observations',
            lambda m: self.observations.append(json.loads(m.data)), 300)
        self.create_subscription(String, '/swarm/relay_events',
            lambda m: self.relay_events.append(json.loads(m.data)), 1000)
        be = lambda d: QoSProfile(depth=d, reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(TargetSighting, '/swarm/target_sightings',
                                 self.shared_sightings.append, be(1))
        self.create_subscription(TargetSighting, '/swarm/target_sightings',
                                 self.shared_deep.append, be(300))
        for i in range(n):
            self.create_subscription(String, f'/agent{i}/target_track_status',
                lambda m, i=i: self.statuses.append(dict(json.loads(m.data), agent=i)), 300)
            self.create_subscription(TargetSighting, f'/agent{i}/local_target_sighting',
                lambda m: self.local_sightings.append(m), 300)


def read_load():
    with open('/proc/loadavg') as f:
        return f.read().split()[:3]


def dump(path, rows):
    path.write_text(''.join(json.dumps(r, sort_keys=True) + '\n' for r in rows))


def run_cell(mode, n, seed, out, domain, comm, radio, tl):
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), SDL_VIDEODRIVER='dummy',
               SDL_AUDIODRIVER='dummy', DIAG_COMM_RANGE=str(comm),
               DIAG_RADIO_RANGE=str(radio))
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    cmd = ['ros2', 'launch', str(LAUNCH), 'headless:=true', 'ui:=false',
           f'num_agents:={n}', 'episodes_max:=1', f'time_limit:={tl}',
           f'seed:={seed}', 'strategy:=intercept', 'game_mode:=ai',
           'perception:=sensor', 'capture_mode:=hull', 'env:=open',
           f'sharing_mode:={mode}']
    rclpy.init(args=None)
    cap = DeepCapture(n)
    load0 = read_load()
    r0 = resource.getrusage(resource.RUSAGE_CHILDREN)
    t0 = time.monotonic()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, env=env, cwd=ROOT / 'ros2_ws')
    try:
        while proc.poll() is None:
            rclpy.spin_once(cap, timeout_sec=0.01)
        wall = time.monotonic() - t0
        for _ in range(8):
            rclpy.spin_once(cap, timeout_sec=0.01)
        log, _ = proc.communicate(timeout=5)
    finally:
        cap.destroy_node(); rclpy.shutdown()
    r1 = resource.getrusage(resource.RUSAGE_CHILDREN)
    load1 = read_load()
    (out / 'launch.log').write_text(log)
    dump(out / 'observations.jsonl', cap.observations)
    dump(out / 'relay_events.jsonl', cap.relay_events)
    dump(out / 'track_status.jsonl', cap.statuses)
    dump(out / 'local_sightings.jsonl', [base.sighting_row(m) for m in cap.local_sightings])
    dump(out / 'shared_sightings_depth1.jsonl', [base.sighting_row(m) for m in cap.shared_sightings])
    dump(out / 'shared_sightings_depth300.jsonl', [base.sighting_row(m) for m in cap.shared_deep])
    m = re.search(r'EPISODE 1 result=(\w+) t=([0-9.]+)s', log)
    cfg = {
        'mode': mode, 'num_agents': n, 'seed': seed, 'ros_domain_id': domain,
        'comm_range': comm, 'radio_range': radio, 'time_limit': tl,
        'command': ' '.join(cmd),
        'env_overrides': {'DIAG_COMM_RANGE': str(comm), 'DIAG_RADIO_RANGE': str(radio)},
        'exit_code': proc.returncode,
        'capture_result': m.group(1) if m else None,
        'capture_time_sec': float(m.group(2)) if m else None,
        'wall_sec': round(wall, 2),
        'children_cpu_sec': round((r1.ru_utime - r0.ru_utime) + (r1.ru_stime - r0.ru_stime), 2),
        'loadavg_before': load0, 'loadavg_after': load1,
        'git_commit': subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True,
                                     capture_output=True).stdout.strip(),
        'traceback_in_log': 'Traceback' in log,
        'metrics': base.metrics(mode, cap),
    }
    cfg['metrics'].pop('matched_local_wire_keys', None)
    (out / 'run_config.json').write_text(json.dumps(cfg, indent=2, sort_keys=True))
    return cfg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--cells', required=True, help='mode:n_agents:seed,...')
    ap.add_argument('--domain', type=int, default=233)
    ap.add_argument('--comm-range', type=float, default=8.0)
    ap.add_argument('--radio-range', type=float, default=8.0)
    ap.add_argument('--time-limit', type=float, default=30.0)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    for cell in a.cells.split(','):
        mode, n, seed = cell.split(':')
        label = f'{mode}-n{n}-seed{seed}'
        c = run_cell(mode, int(n), int(seed), a.out / label, a.domain,
                     a.comm_range, a.radio_range, a.time_limit)
        print(label, c['capture_result'], c['capture_time_sec'], 'wall', c['wall_sec'],
              'cpu', c['children_cpu_sec'], flush=True)


if __name__ == '__main__':
    main()
