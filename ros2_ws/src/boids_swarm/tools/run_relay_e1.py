#!/usr/bin/env python3
"""Phase-2 experiment runner (SDD_relay_phase2_prereg R-04).

Expands a matrix config (relay_e1_matrix.json) into independent runs, executes
them strictly sequentially, and writes one directory per run under --out:
  run_config.json      resolved settings, fingerprint, git commit, validity
  track_status.jsonl.gz  every received /agentK/target_track_status message
  relay/expected_receivers.jsonl  (sim)  per-sighting in-range receiver set
  relay/published_agentK.jsonl    (controllers) keys actually published
  launch.log
A run is "done" once run_config.json exists (valid OR invalid); re-invoking
skips done runs, so an interrupted matrix resumes. Invalid runs (hang, crash,
no episode result) are recorded, never deleted or silently retried.

usage: run_relay_e1.py --config relay_e1_matrix.json --out DIR [--only SUBSTR]
"""
import argparse, gzip, json, os, re, resource, shutil, signal, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import run_sighting_experiments as base
from boids_swarm import experiment_matrix as em

ROOT = base.ROOT
LAUNCH = Path(__file__).resolve().parent / 'diag_pursuit.launch.py'


class StatusCapture(Node):
    """Only the reliable per-agent status topics (carry the receiver-side
    counters); deliberately nothing else, to keep the collector light."""
    def __init__(self, n):
        super().__init__('relay_e1_capture')
        self.statuses = []
        for i in range(n):
            self.create_subscription(
                String, f'/agent{i}/target_track_status',
                lambda m, i=i: self.statuses.append(
                    dict(json.loads(m.data), agent=f'agent{i}')), 500)


class RelayEventCapture(Node):
    """E1b only: every /swarm/relay_events message (reason + age). RELIABLE
    with a deep queue so the collector itself does not drop events."""
    def __init__(self):
        super().__init__('relay_e1b_events')
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.events = []
        self.create_subscription(
            String, '/swarm/relay_events',
            lambda m: self.events.append(json.loads(m.data)),
            QoSProfile(depth=100000, reliability=ReliabilityPolicy.RELIABLE))


def git(*args):
    return subprocess.run(['git', *args], cwd=ROOT, text=True,
                          capture_output=True).stdout.strip()


def read_load():
    with open('/proc/loadavg') as f:
        return f.read().split()[:3]


def build_command(spec, cfg, relay_dir):
    c = cfg['common']
    cmd = ['ros2', 'launch', str(LAUNCH), 'headless:=true', 'ui:=false',
           f'num_agents:={spec.num_agents}',
           f"episodes_max:={c['episodes_max']}",
           f"time_limit:={c['time_limit']}", f'seed:={spec.seed}',
           f"strategy:={c['strategy']}", f"game_mode:={c['game_mode']}",
           f"perception:={c['perception']}",
           f"capture_mode:={c['capture_mode']}", f"env:={c['env']}",
           f'relay_log_dir:={relay_dir}']
    cmd += [f'{k}:={v}' for k, v in spec.launch_args.items()]
    return cmd


def kill_group(proc, sig):
    try:
        os.killpg(proc.pid, sig)
    except ProcessLookupError:
        pass


def run_one(spec, cfg, run_dir, domain, wall_timeout):
    if run_dir.exists():
        shutil.rmtree(run_dir)               # partial leftovers of an interrupted run
    run_dir.mkdir(parents=True)
    relay_dir = run_dir / 'relay'
    relay_dir.mkdir()
    c = cfg['common']
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), SDL_VIDEODRIVER='dummy',
               SDL_AUDIODRIVER='dummy', DIAG_COMM_RANGE=str(c['comm_range']),
               DIAG_RADIO_RANGE=str(c['radio_range']))
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    cmd = build_command(spec, cfg, relay_dir)
    fingerprint = base.source_fingerprint()
    commit = git('rev-parse', 'HEAD')
    dirty = git('status', '--porcelain', '--', 'ros2_ws', 'README.md', 'SDD')
    load0 = read_load()
    r0 = resource.getrusage(resource.RUSAGE_CHILDREN)
    rclpy.init(args=None)
    cap = StatusCapture(spec.num_agents)
    evcap = RelayEventCapture() if cfg.get('capture_relay_events') else None
    t0 = time.monotonic()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env,
                            cwd=ROOT / 'ros2_ws', start_new_session=True)
    wall_timeout_hit = False
    log = ''
    try:
        while proc.poll() is None:
            rclpy.spin_once(cap, timeout_sec=0.01)
            if evcap is not None:
                rclpy.spin_once(evcap, timeout_sec=0)
            if time.monotonic() - t0 > wall_timeout:
                wall_timeout_hit = True
                kill_group(proc, signal.SIGINT)
                time.sleep(3)
                break
        wall = time.monotonic() - t0
        for _ in range(8):
            rclpy.spin_once(cap, timeout_sec=0.01)
            if evcap is not None:
                rclpy.spin_once(evcap, timeout_sec=0.01)
        kill_group(proc, signal.SIGTERM)
        try:
            log, _ = proc.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            kill_group(proc, signal.SIGKILL)
            log, _ = proc.communicate(timeout=5)
    finally:
        cap.destroy_node()
        if evcap is not None:
            evcap.destroy_node()
        rclpy.shutdown()
        kill_group(proc, signal.SIGKILL)     # no straggler survives into next run
    r1 = resource.getrusage(resource.RUSAGE_CHILDREN)
    (run_dir / 'launch.log').write_text(log)
    with gzip.open(run_dir / 'track_status.jsonl.gz', 'wt') as f:
        for row in cap.statuses:
            f.write(json.dumps(row, sort_keys=True) + '\n')
    if evcap is not None:
        with gzip.open(run_dir / 'relay_events.jsonl.gz', 'wt') as f:
            for row in evcap.events:
                f.write(json.dumps(row, sort_keys=True) + '\n')
    m = re.search(r'EPISODE 1 result=(\w+) t=([0-9.]+)s', log)
    validity, reason = em.classify_validity(
        proc.returncode, m is not None, wall_timeout_hit)
    cyc = [s for s in cap.statuses
           if s.get('event') == 'cycle' and s.get('own_pose_fresh')]
    config = {
        'run_id': spec.run_id, 'scene': spec.scene, 'mode': spec.mode,
        'seed': spec.seed, 'num_agents': spec.num_agents,
        'launch_args': spec.launch_args, 'common': c,
        'command': ' '.join(cmd), 'ros_domain_id': domain,
        'validity': validity, 'invalid_reason': reason,
        'exit_code': proc.returncode, 'wall_timeout_hit': wall_timeout_hit,
        'capture_result': m.group(1) if m else None,
        'capture_time_sec': float(m.group(2)) if m else None,
        'captured': bool(m and m.group(1) != 'timeout'),
        'track_valid_fraction': (sum(bool(s['belief_valid']) for s in cyc)
                                 / len(cyc) if cyc else None),
        'track_valid_denominator': len(cyc),
        'status_rows': len(cap.statuses),
        'relay_event_rows': len(evcap.events) if evcap is not None else None,
        'traceback_in_log': 'Traceback' in log,
        'wall_sec': round(wall, 2),
        'children_cpu_sec': round((r1.ru_utime - r0.ru_utime)
                                  + (r1.ru_stime - r0.ru_stime), 2),
        'loadavg_before': load0, 'loadavg_after': read_load(),
        'git_commit': commit, 'working_tree_dirty': bool(dirty),
        'dirty_paths': dirty.splitlines(),
        'source_fingerprint_sha256': fingerprint,
        'finished_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
    }
    (run_dir / 'run_config.json').write_text(
        json.dumps(config, indent=2, sort_keys=True))
    return config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--only', default='', help='run only ids containing this substring')
    ap.add_argument('--domain', type=int, default=None)
    a = ap.parse_args()
    a.out = a.out.resolve()   # launch runs with cwd=ros2_ws: a relative path misplaces relay logs
    cfg = json.loads(a.config.read_text())
    domain = a.domain if a.domain is not None else cfg.get('domain', 201)
    a.out.mkdir(parents=True, exist_ok=True)
    shutil.copy(a.config, a.out / 'matrix_config.json')
    runs = [r for r in em.expand_matrix(cfg) if a.only in r.run_id]
    todo = [r for r in runs if not em.is_done(a.out / r.run_id)]
    print(f'{len(runs)} runs in matrix, {len(runs) - len(todo)} already done, '
          f'{len(todo)} to run; fingerprint {base.source_fingerprint()[:12]} '
          f'commit {git("rev-parse", "--short", "HEAD")}', flush=True)
    for k, spec in enumerate(todo, 1):
        c = run_one(spec, cfg, a.out / spec.run_id, domain,
                    cfg.get('wall_timeout_sec', 240))
        print(f"[{k}/{len(todo)}] {spec.run_id} {c['validity']}"
              f"{'(' + c['invalid_reason'] + ')' if c['invalid_reason'] else ''} "
              f"result={c['capture_result']} t={c['capture_time_sec']} "
              f"wall={c['wall_sec']} tv={c['track_valid_fraction']} "
              f"load={c['loadavg_after'][0]}", flush=True)


if __name__ == '__main__':
    main()
