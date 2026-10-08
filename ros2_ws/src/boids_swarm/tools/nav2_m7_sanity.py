#!/usr/bin/env python3
"""M7 sanity benchmark (NOT a pre-registered experiment).

12 agents, intercept, env=obstacle_field, evader reactive vs nav2, seeds 1-6
(all of them), a 30 s ACTIVE window per run. Both evaders see the same launch
(including `warmup`, which holds the controllers back until Nav2 is active,
and time_scale 1.0 because Nav2 runs on wall-clock CPU). The harness, not
the sim, enforces the 30 s window: it starts counting sim time when the
first boid moves and kills the launch if there is no capture by then.

    python3 tools/nav2_m7_sanity.py --seeds 1,2,3,4,5,6 --out-dir artifacts/nav2-m7-sanity
"""

import argparse
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray, String
from turtlesim.msg import Pose

EP_RE = re.compile(r'EPISODE (\d+) result=(\w+) t=([\d.]+)s')


class Recorder(Node):
    def __init__(self):
        super().__init__('sanity_recorder')
        self.reset()
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(Pose, '/target/pose', self._pose, 10)
        self.create_subscription(Float32MultiArray, '/swarm/poses',
                                 self._swarm, 10)
        self.create_subscription(String, '/target/evader_status',
                                 self._status, 10)
        self.create_subscription(Twist, '/target/cmd_vel', self._on_cmd, 10)

    def reset(self):
        self.sim_t = 0.0
        self.t_boid = None         # sim time the first boid moved
        self.t_cmd = None          # sim time of the first /target/cmd_vel
        self.speeds = []           # target speed samples within the window
        self.speed_t = []          # their sim times
        self.status = None
        self.status_log = []       # (sim time, mode) of every /target/evader_status

    def _on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    @property
    def t0(self):
        """Start of the ACTIVE window = first boid movement, valid only if
        the target controller was already publishing (the launch holds the
        boids back by `pursuer_delay`; otherwise the slower-starting Nav2
        target process would hand the pursuers a timing-dependent head
        start, and the run is retried)."""
        if self.t_boid is None or self.t_cmd is None \
                or self.t_cmd > self.t_boid:      # target not up before the chase
            return None
        return self.t_boid

    def _on_cmd(self, _m):
        if self.t_cmd is None:
            self.t_cmd = self.sim_t

    def _swarm(self, m):
        d = m.data
        if self.t_boid is None and len(d) and any(
                abs(d[4 + 5 * i]) > 0.05 for i in range(int(d[0]))):
            self.t_boid = self.sim_t

    def _pose(self, p):
        if self.t0 is not None:
            self.speeds.append(p.linear_velocity)
            self.speed_t.append(self.sim_t)

    def _status(self, m):
        self.status = json.loads(m.data)
        self.status_log.append((self.sim_t, self.status.get('mode')))

    def speed_by_mode(self, t1):
        """Mean target speed per evader mode, plus the share of samples with
        speed < 0.3 m/s (standing / pivoting)."""
        import bisect
        times = [t for t, _ in self.status_log]
        acc = {'nav2': [], 'blend': [], 'reactive': []}
        slow = 0
        for t, v in zip(self.speed_t, self.speeds):
            if t > t1:
                break
            slow += v < 0.3
            i = bisect.bisect_right(times, t) - 1
            if i >= 0 and self.status_log[i][1] in acc:
                acc[self.status_log[i][1]].append(v)
        n = max(1, sum(1 for t in self.speed_t if t <= t1))
        return ({k: (round(statistics.mean(v), 2) if v else None)
                 for k, v in acc.items()}, round(slow / n, 3))

    def mode_seconds(self, t0, t1):
        """TIME-weighted seconds per mode inside [t0, t1]: each status
        message's mode holds until the next message (status is published
        every 0.1 s of sim time; gaps > 1 s are not credited to a mode)."""
        out = {'nav2': 0.0, 'blend': 0.0, 'reactive': 0.0}
        log = self.status_log
        for (ta, mode), (tb, _) in zip(log, log[1:]):
            a, b = max(ta, t0), min(tb, t1)
            if b > a and tb - ta <= 1.0 and mode in out:
                out[mode] += b - a
        return out


def one_run(rec, evader, seed, active, warmup, delay, outdir):
    rec.reset()
    log = os.path.join(outdir, f'{evader}_s{seed}.log')
    f = open(log, 'w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'boids_swarm', 'pursuit.launch.py',
         'headless:=true', 'ui:=false', 'num_agents:=12',
         'strategy:=intercept', 'env:=obstacle_field', f'seed:={seed}',
         f'evader:={evader}', 'episodes_max:=1', 'time_limit:=300',
         'time_scale:=1.0', f'warmup:={warmup}', f'pursuer_delay:={delay}'],
        stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    wall0 = time.monotonic()
    result, t_cap = 'timeout', None
    try:
        while True:
            time.sleep(0.2)
            if proc.poll() is not None:
                break
            if time.monotonic() - wall0 > active + warmup + 90:
                result = 'harness_wall_timeout'
                break
            if rec.t0 is not None and rec.sim_t - rec.t0 >= active:
                break
    finally:
        sim_end = rec.sim_t
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        f.close()
    txt = open(log, errors='replace').read()
    m = EP_RE.search(txt)
    t0 = rec.t0
    if m and m.group(2) == 'captured':
        result = 'captured'
        # sim episode clock started with the sim; convert to active window
        t_cap = None if t0 is None else float(m.group(3)) - t0
    out = {'evader': evader, 'seed': seed, 'result': result,
           'window_unknown': t0 is None,
           't_boid_moved': rec.t_boid, 't_target_cmd': rec.t_cmd,
           'capture_time_s': None if t_cap is None else round(max(t_cap, 0), 2),
           'active_s': None if t0 is None else round(min(sim_end - t0, active), 2),
           'target_mean_speed_mps': round(statistics.mean(rec.speeds), 2)
           if rec.speeds else None}
    st = rec.status
    if st and t0 is not None:
        secs = rec.mode_seconds(t0, min(sim_end, t0 + active))
        tot_s = max(sum(secs.values()), 1e-9)
        tot = max(1, st['ticks_nav2'] + st['ticks_blend'] + st['ticks_reactive'])
        req, ok = st['plan_requests'], st['plan_ok']
        sbm, slow = rec.speed_by_mode(min(sim_end, t0 + active))
        out.update(
            speed_by_mode_mps=sbm, slow_sample_share=slow,
            # time-weighted (status timestamps); the per-tick ratio below is
            # kept only to show how much the two differ
            mode_time_ratio={k: round(v / tot_s, 3) for k, v in secs.items()},
            mode_tick_ratio={k: round(st['ticks_' + k] / tot, 3)
                             for k in ('nav2', 'blend', 'reactive')},
            nav2_plan_requests=req,
            nav2_plan_fail=st['plan_fail'],
            nav2_plan_timeout=st.get('plan_timeout', 0),
            nav2_plan_fail_rate=round(st['plan_fail'] / req, 3) if req else None,
            # follow aborts are a different failure (counted per FollowPath
            # started = plan_ok), not mixed into the plan failure rate
            nav2_follow_started=ok,
            nav2_follow_abort=st['follow_abort'],
            nav2_follow_abort_rate=round(st['follow_abort'] / ok, 3) if ok else None,
            nav2_goals=st['goals'])
    time.sleep(3.0)
    return out


def aggregate(rows):
    cap = [r for r in rows if r['result'] == 'captured']
    cap_t = [r for r in cap if r['capture_time_s'] is not None]
    sp = [r['target_mean_speed_mps'] for r in rows
          if r['target_mean_speed_mps'] is not None]
    agg = {'runs': len(rows), 'captures': len(cap),
           'capture_rate': round(len(cap) / len(rows), 2) if rows else None,
           'mean_capture_time_s': round(statistics.mean(
               r['capture_time_s'] for r in cap_t), 2) if cap_t else None,
           'runs_with_unknown_window': len(cap) - len(cap_t),
           'mean_target_speed_mps': round(statistics.mean(sp), 2) if sp else None}
    withm = [r for r in rows if 'mode_time_ratio' in r]
    if withm:
        agg['mode_time_ratio_mean'] = {
            k: round(statistics.mean(r['mode_time_ratio'][k] for r in withm), 3)
            for k in ('nav2', 'blend', 'reactive')}
        req = sum(r['nav2_plan_requests'] for r in withm)
        agg['nav2_plan_requests_total'] = req
        agg['nav2_plan_fail_total'] = sum(r['nav2_plan_fail'] for r in withm)
        agg['nav2_plan_fail_rate'] = round(
            agg['nav2_plan_fail_total'] / req, 3) if req else None
        agg['nav2_follow_started_total'] = sum(r['nav2_follow_started'] for r in withm)
        agg['nav2_follow_abort_total'] = sum(r['nav2_follow_abort'] for r in withm)
    return agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', default='1,2,3,4,5,6',
                    help='all seeds; none is skipped (seed 1 spawns the target '
                         'close to the swarm and is captured early)')
    ap.add_argument('--active', type=float, default=30.0)
    ap.add_argument('--warmup', type=float, default=8.0)
    ap.add_argument('--delay', type=float, default=4.0,
                    help='seconds the boids wait after the target controller')
    ap.add_argument('--evaders', default='reactive,nav2')
    ap.add_argument('--max-attempts', type=int, default=3)
    ap.add_argument('--out-dir', default='artifacts/nav2-m7-sanity')
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    logs = os.path.join(a.out_dir, 'logs')
    os.makedirs(logs, exist_ok=True)
    rclpy.init()
    rec = Recorder()
    ex = SingleThreadedExecutor()
    ex.add_node(rec)
    threading.Thread(target=ex.spin, daemon=True).start()
    seeds = [int(x) for x in a.seeds.split(',')]
    rows, discarded = [], []
    for evader in a.evaders.split(','):
        for seed in seeds:
            for attempt in range(1, a.max_attempts + 1):
                r = one_run(rec, evader, seed, a.active, a.warmup, a.delay, logs)
                r['attempt'] = attempt
                print(json.dumps(r), flush=True)
                # a run in which the target controller never published before
                # the end (captured while still starting up) has no defined
                # active window: it is a harness failure, not a game outcome
                if not r['window_unknown']:
                    break
                discarded.append(r)
            rows.append(r)
    by = {e: aggregate([r for r in rows if r['evader'] == e])
          for e in a.evaders.split(',')}
    git = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'],
                         capture_output=True, text=True).stdout.strip()
    summary = {'kind': 'sanity (not pre-registered)', 'git_head': git,
               'config': {'num_agents': 12, 'strategy': 'intercept',
                          'env': 'obstacle_field', 'seeds': seeds,
                          'active_window_s': a.active, 'warmup_s': a.warmup, 'pursuer_delay_s': a.delay,
                          'time_scale': 1.0, 'stamina': True,
                          'capture_mode': 'hull'},
               'aggregate': by, 'runs': rows,
               'discarded_startup_failures': discarded}
    with open(os.path.join(a.out_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(by, indent=1))
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
