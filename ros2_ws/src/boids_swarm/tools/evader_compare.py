#!/usr/bin/env python3
"""Evader behaviour comparison (SANITY CHECK, not a pre-registered experiment).

Runs a headless 12-agent chase (episode 2 of 2; episode 1 is warm-up, see run_one) per (evader, env, seed) on CPU, records the
target's track and reports how much of the time it spends on a wall / obstacle,
its mean speed, whether and when it was captured, and how often it got stuck.

    # one run (writes JSON):
    evader_compare.py run --evader smart --env obstacle_field --seed 3 \\
        --domain 170 --out run.json
    # a sweep, N runs in parallel on distinct ROS domains, then summary:
    evader_compare.py sweep --evaders reactive,adaptive,smart \\
        --envs obstacle_field,open --seeds 1-10 --out DIR --domain-base 170 \\
        --jobs 6
    evader_compare.py summarize --out DIR

Definitions (pose = target centre, br = target body radius from params.yaml):
  touch  = distance from the pose to the wall / obstacle surface <= br + 0.03
           (the sim clamps the body exactly there)
  near   = distance from the pose to the surface <= 0.73 (the wider band the
           earlier diagnostic recorder used, kept for comparability)
  stuck  = a run of >= 2.0 s (sim time) with speed < 0.5
Only the target's own pose and the sim clock are read; nothing is changed.
"""
import argparse
import json
import math
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

ARENA = 20.0
BR = 0.15                  # target_body_radius in config/params.yaml
TOUCH_TOL = 0.03
NEAR_BAND = 0.73
STUCK_SPEED, STUCK_SECONDS = 0.5, 2.0


def _surface_dist(x, y, obstacles):
    w = min(x, ARENA - x, y, ARENA - y)
    o = min((math.hypot(x - cx, y - cy) - r for cx, cy, r in obstacles),
            default=math.inf)
    return w, o


def metrics(track, obstacles):
    """track: [(t, x, y, v)] -> behaviour metrics."""
    n = len(track)
    if n == 0:
        return {}
    wt = ot = wn = on = 0
    stuck_events, stuck_s, run_start = 0, 0.0, None
    for k, (t, x, y, v) in enumerate(track):
        w, o = _surface_dist(x, y, obstacles)
        wt += w <= BR + TOUCH_TOL
        ot += o <= BR + TOUCH_TOL
        wn += w <= NEAR_BAND
        on += o <= NEAR_BAND
        if v < STUCK_SPEED:
            if run_start is None:
                run_start = t
        else:
            if run_start is not None and t - run_start >= STUCK_SECONDS:
                stuck_events += 1
                stuck_s += t - run_start
            run_start = None
    if run_start is not None and track[-1][0] - run_start >= STUCK_SECONDS:
        stuck_events += 1
        stuck_s += track[-1][0] - run_start
    return {'n': n, 'wall_touch': wt / n, 'obs_touch': ot / n,
            'wall_near': wn / n, 'obs_near': on / n,
            'mean_speed': sum(p[3] for p in track) / n,
            'stuck_events': stuck_events, 'stuck_s': stuck_s,
            't_first': track[0][0],
            'duration': track[-1][0] - track[0][0]}


# ------------------------------------------------------------------ one run
def run_one(a):
    os.environ['ROS_DOMAIN_ID'] = str(a.domain)
    os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
    import rclpy
    from rclpy.node import Node
    from boids_swarm_msgs.msg import EpisodeState
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from turtlesim.msg import Pose
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from boids_swarm.world_gen import WorldGenerator
    obstacles = WorldGenerator(a.seed, ARENA).generate(a.env).obstacles

    if a.smart_json:
        os.environ['SMART_PARAMS_JSON'] = a.smart_json
    log = open(str(a.out) + '.log', 'w')
    extra = [f'time_scale:={a.time_scale}'] if a.time_scale else []
    cmd = ['ros2', 'launch', 'boids_swarm', 'pursuit.launch.py',
           f'num_agents:={a.agents}', f'env:={a.env}', f'seed:={a.seed}',
           f'evader:={a.evader}', 'headless:=true', 'ui:=false',
           'episodes_max:=2', f'time_limit:={a.time_limit}',
           'perception:=perfect', 'sharing_mode:=legacy'] + extra

    class Rec(Node):
        def __init__(s):
            super().__init__('evader_compare_rec')
            s.t = 0.0
            s.track = []
            s.last = None
            s.ep_starts = []
            s.create_subscription(Clock, '/clock', s.on_clock, 10)
            s.create_subscription(
                EpisodeState, '/simulation/episode_state',
                lambda m: s.ep_starts.append(
                    m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            s.create_subscription(Pose, '/target/pose', s.on_pose, 50)

        def on_clock(s, m):
            s.t = m.clock.sec + m.clock.nanosec * 1e-9

        def on_pose(s, m):
            s.last = time.time()
            s.track.append((s.t, m.x, m.y, abs(m.linear_velocity)))

    rclpy.init()
    node = Rec()
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if proc.poll() is not None:
                break
            if node.last and time.time() - node.last > 5.0:
                break
            if time.time() - t0 > a.wall_timeout:
                break
    finally:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
    text = open(str(a.out) + '.log').read()
    # Episode 1 is a warm-up: the sim starts its clock while the controller
    # processes are still importing, so only the respawned episode 2 is a
    # fair, synchronised chase. Its first pose is the last teleport (> 2 m
    # in one sample, impossible at <= 3.6 m/s) in the received track; if the
    # recorder joined after the respawn, fall back to the episode-state stamp.
    raw = node.track
    cut = None
    for k in range(1, len(raw)):
        if math.hypot(raw[k][1] - raw[k - 1][1],
                      raw[k][2] - raw[k - 1][2]) > 2.0:
            cut = k
    if cut is not None:
        track = raw[cut:]
    elif node.ep_starts and node.ep_starts[-1] > 0.5:
        track = [p for p in raw if p[0] >= node.ep_starts[-1]]
    else:
        track = []
    t0 = track[0][0] if track else 0.0
    track = [(t - t0, x, y, v) for t, x, y, v in track]
    m = re.search(r'EPISODE 2 result=(\w+) t=([\d.]+)s', text)
    res = metrics(track, obstacles)
    res['track2hz'] = [[round(v, 2) for v in p] for p in node.track[::15]]
    res.update(evader=a.evader, env=a.env, seed=a.seed,
               valid=bool(m) and bool(track),
               captured=bool(m and m.group(1) == 'captured'),
               capture_t=(float(m.group(2)) if m and m.group(1) == 'captured'
                          else None))
    Path(a.out).write_text(json.dumps(res))
    print(json.dumps(res))


# ------------------------------------------------------------------- sweep
def parse_seeds(s):
    out = []
    for part in s.split(','):
        if '-' in part:
            lo, hi = part.split('-')
            out += list(range(int(lo), int(hi) + 1))
        else:
            out.append(int(part))
    return out


def sweep(a):
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    jobs = [(e, env, s) for env in a.envs.split(',')
            for e in a.evaders.split(',') for s in parse_seeds(a.seeds)]
    todo = [j for j in jobs
            if not (out / f'{j[0]}_{j[1]}_{j[2]}.json').exists()]
    running = {}                          # slot -> (Popen, job)
    slots = list(range(a.jobs))
    while todo or running:
        while todo and slots:
            slot = slots.pop()
            e, env, s = todo.pop(0)
            p = subprocess.Popen(
                [sys.executable, __file__, 'run', '--evader', e, '--env', env,
                 '--seed', str(s), '--domain', str(a.domain_base + slot),
                 '--agents', str(a.agents), '--time-limit', str(a.time_limit),
                 '--time-scale', a.time_scale,
                 '--smart-json', a.smart_json,
                 '--out', str(out / f'{e}_{env}_{s}.json')],
                stdout=subprocess.DEVNULL)
            running[slot] = (p, (e, env, s))
        time.sleep(1.0)
        for slot, (p, job) in list(running.items()):
            if p.poll() is not None:
                del running[slot]
                slots.append(slot)
                print('done', job, flush=True)


def _stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return {'median': statistics.median(vals), 'min': min(vals),
            'max': max(vals), 'n': len(vals)}


def _pooled(rs, keys):
    """Time-weighted: all samples of all runs together (long runs count
    more), summed over the given wall/obstacle fractions."""
    n = sum(r['n'] for r in rs)
    return (sum(r['n'] * sum(r[k] for k in keys) for r in rs) / n) if n else None


def summarize(a):
    out = Path(a.out)
    runs = [json.loads(p.read_text()) for p in sorted(out.glob('*_*_*.json'))
            if p.name != 'summary.json']
    table = {}
    for r in runs:
        table.setdefault((r['env'], r['evader']), []).append(r)
    summary = {}
    for (env, ev), rs in sorted(table.items()):
        good = [r for r in rs if r['valid']]
        caps = [r['capture_t'] for r in good if r['captured']]
        summary[f'{env}/{ev}'] = {
            'runs': len(rs), 'valid': len(good),
            'pooled_touch': _pooled(good, ('wall_touch', 'obs_touch')),
            'pooled_near': _pooled(good, ('wall_near', 'obs_near')),
            'wall_touch': _stats([r['wall_touch'] for r in good]),
            'obs_touch': _stats([r['obs_touch'] for r in good]),
            'touch_sum': _stats([r['wall_touch'] + r['obs_touch']
                                 for r in good]),
            'wall_near': _stats([r['wall_near'] for r in good]),
            'obs_near': _stats([r['obs_near'] for r in good]),
            'near_sum': _stats([r['wall_near'] + r['obs_near']
                                for r in good]),
            'mean_speed': _stats([r['mean_speed'] for r in good]),
            'captures': len(caps),
            'capture_t': _stats(caps),
            'stuck_events_total': sum(r['stuck_events'] for r in good),
            'stuck_runs': sum(1 for r in good if r['stuck_events']),
        }
    Path(out / 'summary.json').write_text(json.dumps(summary, indent=1))
    pct = lambda s: ('-' if s is None else
                     f"{100*s['median']:.0f}% ({100*s['min']:.0f}-{100*s['max']:.0f})")
    print('env/evader | n | touch(w+o) med(range) | near(w+o) med(range) | '
          'pooled touch/near | speed | caps | cap_t med | stuck')
    for k, v in summary.items():
        ct = v['capture_t']
        print(f"{k} | {v['valid']}/{v['runs']} | {pct(v['touch_sum'])} | "
              f"{pct(v['near_sum'])} | "
              f"{100*v['pooled_touch']:.0f}%/{100*v['pooled_near']:.0f}% | "
              f"{v['mean_speed']['median']:.2f} | {v['captures']} | "
              f"{'-' if ct is None else format(ct['median'], '.1f')} | "
              f"{v['stuck_events_total']} in {v['stuck_runs']} runs")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--evader', required=True)
    r.add_argument('--env', required=True)
    r.add_argument('--seed', type=int, required=True)
    r.add_argument('--domain', type=int, required=True)
    r.add_argument('--agents', type=int, default=12)
    r.add_argument('--time-limit', type=float, default=30.0)
    r.add_argument('--time-scale', default='',
                   help='sim fast-forward (empty = sim default 4)')
    r.add_argument('--smart-json', default='',
                   help='JSON of smart_<field> overrides (tuning hook)')
    r.add_argument('--wall-timeout', type=float, default=150.0)
    r.add_argument('--out', required=True)
    s = sub.add_parser('sweep')
    s.add_argument('--evaders', default='reactive,adaptive,smart')
    s.add_argument('--envs', default='obstacle_field,open')
    s.add_argument('--seeds', default='1-10')
    s.add_argument('--agents', type=int, default=12)
    s.add_argument('--time-limit', type=float, default=30.0)
    s.add_argument('--time-scale', default='')
    s.add_argument('--smart-json', default='')
    s.add_argument('--domain-base', type=int, default=170)
    s.add_argument('--jobs', type=int, default=6)
    s.add_argument('--out', required=True)
    m = sub.add_parser('summarize')
    m.add_argument('--out', required=True)
    a = ap.parse_args()
    {'run': run_one, 'sweep': sweep, 'summarize': summarize}[a.cmd](a)


if __name__ == '__main__':
    main()
