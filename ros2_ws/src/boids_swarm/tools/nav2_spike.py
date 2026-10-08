#!/usr/bin/env python3
"""M7 feasibility spike: can Nav2 (Jazzy) steer the 2x target around circular
obstacles in the pygame sim, and at what latency?  (SDD v3 §6.4, approach B)

Per configuration it starts pygame_sim (headless, 2 static boids, wall-clock
1x) and nav2_target.launch.py, then measures:
  - lifecycle bring-up time (launch -> planner_server AND controller_server active)
  - ComputePathToPose latency (N calls: mean / max)
  - FollowPath: reached goal?, time, controller output rate, min clearance
    to obstacles (< 0 means the target body touched an obstacle) and to boids.

Run (needs the workspace sourced, ROS_DOMAIN_ID set, pygame importable):
    export ROS_DOMAIN_ID=232 SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy
    python3 tools/nav2_spike.py --out artifacts/nav2-spike/result.json
"""

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray
from turtlesim.msg import Pose

BODY_R = 0.15
WORLD = 20.0
V_MAX = 4.0      # agent_max_speed 2.0 x multiplier 2.0
W_MAX = 1.2      # params.yaml target_omega_max


# ---------------------------------------------------------------- scenarios
def _unit(ax, ay, bx, by):
    d = math.hypot(bx - ax, by - ay)
    return (bx - ax) / d, (by - ay) / d, d


def pick_goal(sx, sy):
    corners = [(3.5, 3.5), (16.5, 3.5), (3.5, 16.5), (16.5, 16.5)]
    return max(corners, key=lambda c: math.hypot(c[0] - sx, c[1] - sy))


def build_config(name, sx, sy):
    gx, gy = pick_goal(sx, sy)
    ux, uy, dist = _unit(sx, sy, gx, gy)
    nx, ny = -uy, ux
    mx, my = (sx + gx) / 2, (sy + gy) / 2
    obs = []
    if name == 'single_block':
        obs = [(mx, my, 2.0)]
    elif name == 'wall_with_gap':
        # perpendicular wall of r=1.3 circles at the midpoint; one is left out
        for k in range(-4, 5):
            if k == 2:
                continue
            obs.append((mx + nx * k * 2.2, my + ny * k * 2.2, 1.3))
    elif name == 'slalom':
        for f, side in ((0.28, 1), (0.5, -1), (0.72, 1)):
            px, py = sx + ux * dist * f, sy + uy * dist * f
            obs.append((px + nx * 1.0 * side, py + ny * 1.0 * side, 1.5))
    else:
        raise ValueError(name)
    # keep only obstacles that do not swallow start/goal
    obs = [o for o in obs
           if math.hypot(o[0] - sx, o[1] - sy) > o[2] + 1.0
           and math.hypot(o[0] - gx, o[1] - gy) > o[2] + 1.0]
    return {'name': name, 'start': (sx, sy), 'goal': (gx, gy), 'obstacles': obs}


CONFIGS = [('single_block', 11), ('wall_with_gap', 5), ('slalom', 23)]


# ---------------------------------------------------------------- harness
class Spike(Node):
    def __init__(self):
        super().__init__('nav2_spike')
        self.pose = None
        self.boids = []
        self.sim_time = 0.0
        self.samples = []            # (wall, sim, x, y, theta, v)
        self.cmd_wall = []
        self.recording = False
        self.create_subscription(Pose, '/target/pose', self._on_pose, 10)
        self.create_subscription(Float32MultiArray, '/swarm/poses', self._on_swarm, 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)
        self.create_subscription(Twist, '/target/cmd_vel', self._on_cmd, 50)
        self.plan_ac = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_ac = ActionClient(self, FollowPath, 'follow_path')
        self.setp = self.create_client(SetParameters, '/pygame_sim/set_parameters')

    def _on_clock(self, m):
        self.sim_time = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_pose(self, p):
        self.pose = p
        if self.recording:
            self.samples.append((time.monotonic(), self.sim_time, p.x, p.y,
                                 p.theta, p.linear_velocity))

    def _on_swarm(self, m):
        d = m.data
        n = int(d[0]) if d else 0
        self.boids = [(d[1 + 5 * i], d[2 + 5 * i]) for i in range(n)]

    def _on_cmd(self, _):
        if self.recording:
            self.cmd_wall.append(time.monotonic())

    def wait_for(self, cond, timeout, what):
        t0 = time.monotonic()
        while not cond():
            if time.monotonic() - t0 > timeout:
                raise TimeoutError(what)
            time.sleep(0.002)

    def set_sim_obstacles(self, obs):
        flat = [float(v) for o in obs for v in o] or [0.0]
        req = SetParameters.Request()
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                            double_array_value=flat)
        req.parameters = [Parameter(name='obstacles', value=pv)]
        self.setp.wait_for_service(5.0)
        fut = self.setp.call_async(req)
        self.wait_for(fut.done, 5.0, 'set obstacles')
        assert fut.result().results[0].successful

    def lifecycle_active(self, node):
        cli = self.create_client(GetState, f'/{node}/get_state')
        if not cli.service_is_ready():
            return False
        fut = cli.call_async(GetState.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 1.0:
            time.sleep(0.01)
        ok = fut.done() and fut.result().current_state.id == 3
        self.destroy_client(cli)
        return ok


def _pose_stamped(msg_cls, x, y, stamp):
    ps = msg_cls()
    ps.header.frame_id = 'map'
    ps.header.stamp = stamp
    ps.pose.position.x, ps.pose.position.y = float(x), float(y)
    ps.pose.orientation.w = 1.0
    return ps


def spawn(cmd, log, env):
    f = open(log, 'w')
    return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env,
                            start_new_session=True)


def stop(proc):
    if proc and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)


def clearance_stats(samples, obs, boids):
    min_obs, min_boid = float('inf'), float('inf')
    for (_, _, x, y, _, _) in samples:
        for (cx, cy, r) in obs:
            min_obs = min(min_obs, math.hypot(x - cx, y - cy) - r - BODY_R)
        for (bx, by) in boids:
            min_boid = min(min_boid, math.hypot(x - bx, y - by))
    # arena walls
    wall = min((min(x, y, WORLD - x, WORLD - y) - BODY_R
                for (_, _, x, y, _, _) in samples), default=float('inf'))
    return min_obs, min_boid, wall


def path_clearance(path, obs):
    best = float('inf')
    for ps in path.poses:
        for (cx, cy, r) in obs:
            best = min(best, math.hypot(ps.pose.position.x - cx,
                                        ps.pose.position.y - cy) - r - BODY_R)
    return best


def run_config(node, name, seed, outdir, n_plans, follow_timeout):
    env = dict(os.environ)
    res = {'config': name, 'seed': seed}
    sim = nav = None
    try:
        sim = spawn(['ros2', 'run', 'boids_swarm', 'pygame_sim', '--ros-args',
                     '-p', 'headless:=true', '-p', 'ui_enabled:=false',
                     '-p', 'num_agents:=2', '-p', 'time_scale:=1.0',
                     '-p', f'seed:={seed}',
                     '-p', 'target_speed_multiplier:=2.0',
                     '-p', f'target_omega_max:={W_MAX}',
                     '-p', f'target_body_radius:={BODY_R}',
                     '-p', 'episode_time_limit:=0.0', '-p', 'auto_reset:=false'],
                    os.path.join(outdir, f'{name}_sim.log'), env)
        node.pose = None
        node.wait_for(lambda: node.pose is not None, 30, 'sim /target/pose')
        time.sleep(0.5)
        cfg = build_config(name, node.pose.x, node.pose.y)
        res.update(start=cfg['start'], goal=cfg['goal'], obstacles=cfg['obstacles'],
                   boids=list(node.boids))
        node.set_sim_obstacles(cfg['obstacles'])
        obs_arg = ';'.join(f'{x:.3f},{y:.3f},{r:.3f}' for x, y, r in cfg['obstacles'])

        t_launch = time.monotonic()
        nav = spawn(['ros2', 'launch', 'boids_swarm', 'nav2_target.launch.py',
                     f'obstacles:={obs_arg}'],
                    os.path.join(outdir, f'{name}_nav2.log'), env)
        node.wait_for(lambda: node.lifecycle_active('planner_server')
                      and node.lifecycle_active('controller_server'),
                      90, 'lifecycle active')
        res['lifecycle_active_s'] = round(time.monotonic() - t_launch, 2)
        node.wait_for(lambda: node.plan_ac.wait_for_server(0.1)
                      and node.follow_ac.wait_for_server(0.1), 20, 'action servers')
        time.sleep(2.0)      # let costmaps (global 2 Hz / local 10 Hz) fill

        gx, gy = cfg['goal']
        lat, last_path, plan_err = [], None, None
        for _ in range(n_plans):
            g = ComputePathToPose.Goal()
            g.goal = _pose_stamped(type(g.goal), gx, gy, node.get_clock().now().to_msg())
            g.use_start = False
            g.planner_id = 'GridBased'
            t0 = time.monotonic()
            fut = node.plan_ac.send_goal_async(g)
            node.wait_for(fut.done, 10, 'plan goal accepted')
            rfut = fut.result().get_result_async()
            node.wait_for(rfut.done, 10, 'plan result')
            dt = time.monotonic() - t0
            r = rfut.result()
            if r.status == 4 and len(r.result.path.poses) > 1:
                lat.append(dt)
                last_path = r.result.path
            else:
                plan_err = f'status={r.status} error_code={r.result.error_code}'
            time.sleep(0.1)
        res['plan_ok'] = len(lat)
        res['plan_calls'] = n_plans
        if plan_err:
            res['plan_error'] = plan_err
        if lat:
            res['plan_latency_ms_mean'] = round(1000 * statistics.mean(lat), 1)
            res['plan_latency_ms_max'] = round(1000 * max(lat), 1)
            res['plan_latency_ms_first'] = round(1000 * lat[0], 1)
        if last_path is None:
            res['reached'] = False
            res['failure'] = 'no path: ' + (plan_err or '')
            return res
        res['path_points'] = len(last_path.poses)
        res['path_min_clearance'] = round(path_clearance(last_path, cfg['obstacles']), 3)
        plen = sum(math.hypot(a.pose.position.x - b.pose.position.x,
                              a.pose.position.y - b.pose.position.y)
                   for a, b in zip(last_path.poses, last_path.poses[1:]))
        res['path_length_m'] = round(plen, 2)
        res['straight_dist_m'] = round(math.hypot(gx - cfg['start'][0], gy - cfg['start'][1]), 2)

        node.samples.clear()
        node.cmd_wall.clear()
        node.recording = True
        f = FollowPath.Goal()
        f.path = last_path
        f.controller_id = 'FollowPath'
        f.goal_checker_id = 'general_goal_checker'
        t_follow, sim_follow = time.monotonic(), node.sim_time
        fut = node.follow_ac.send_goal_async(f)
        node.wait_for(fut.done, 10, 'follow goal accepted')
        gh = fut.result()
        if not gh.accepted:
            res['reached'] = False
            res['failure'] = 'FollowPath goal rejected'
            return res
        rfut = gh.get_result_async()
        try:
            node.wait_for(rfut.done, follow_timeout, 'follow result')
            status = rfut.result().status
            res['follow_status'] = status
            res['follow_error_code'] = rfut.result().result.error_code \
                if hasattr(rfut.result().result, 'error_code') else None
        except TimeoutError:
            gh.cancel_goal_async()
            status = None
            res['follow_status'] = 'timeout'
        node.recording = False
        end = node.samples[-1] if node.samples else None
        dist_end = math.hypot(end[2] - gx, end[3] - gy) if end else float('nan')
        res['final_goal_dist_m'] = round(dist_end, 3)
        res['reached'] = bool(status == 4 and dist_end < 0.6)
        res['follow_wall_s'] = round((end[0] if end else time.monotonic()) - t_follow, 2)
        res['follow_sim_s'] = round((end[1] if end else node.sim_time) - sim_follow, 2)
        res['follow_error'] = None
        min_obs, min_boid, wall = clearance_stats(node.samples, cfg['obstacles'], node.boids)
        res['min_obstacle_clearance_m'] = round(min_obs, 3)
        res['min_boid_dist_m'] = round(min_boid, 3)
        res['min_wall_clearance_m'] = round(wall, 3)
        res['collided_obstacle'] = bool(min_obs < 0.02)
        if node.samples:
            res['max_speed_mps'] = round(max(s[5] for s in node.samples), 2)
            res['mean_speed_mps'] = round(statistics.mean(s[5] for s in node.samples), 2)
        if len(node.cmd_wall) > 5:
            span = node.cmd_wall[-1] - node.cmd_wall[0]
            res['controller_hz'] = round((len(node.cmd_wall) - 1) / span, 1)
        return res
    except Exception as e:             # keep going with the next config
        res.setdefault('reached', False)
        res['failure'] = f'{type(e).__name__}: {e}'
        return res
    finally:
        node.recording = False
        stop(nav)
        stop(sim)
        time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='artifacts/nav2-spike/result.json')
    ap.add_argument('--only', default='')
    ap.add_argument('--plans', type=int, default=10)
    ap.add_argument('--follow-timeout', type=float, default=60.0)
    a = ap.parse_args()
    outdir = os.path.dirname(os.path.abspath(a.out))
    os.makedirs(outdir, exist_ok=True)
    rclpy.init()
    node = Spike()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    results = []
    for name, seed in CONFIGS:
        if a.only and name != a.only:
            continue
        print(f'== {name} (seed {seed})', flush=True)
        r = run_config(node, name, seed, outdir, a.plans, a.follow_timeout)
        print(json.dumps(r, indent=1), flush=True)
        results.append(r)
    summary = {'v_max': V_MAX, 'w_max': W_MAX, 'results': results}
    with open(a.out, 'w') as f:
        json.dump(summary, f, indent=2)
    print('wrote', a.out)
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
