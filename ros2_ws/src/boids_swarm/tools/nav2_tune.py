#!/usr/bin/env python3
"""M7 controller tuning bench: obstacle-course speed with NO boids.

Brings up pygame_sim (headless, 0 boids, stamina OFF so the controller's
own capability is measured) plus nav2_target.launch.py with a given yaml,
then drives the target through several scenarios by sending
ComputePathToPose + FollowPath directly. Reports per leg: reached, mean
speed (path length / sim time), min obstacle clearance, stall (any 2 s
window < 0.3 m/s before arrival).

Limits come from params.yaml like in the real launch (3.6 m/s, 1.2 rad/s).

    python3 tools/nav2_tune.py --config /path/to.yaml --label mppi_v3 \\
        --out artifacts/nav2-m7-tuning/mppi_v3.json
"""

import argparse
import json
import math
import os
import statistics
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav2_spike as sp                       # noqa: E402  (reuse harness)
import rclpy                                   # noqa: E402
from nav2_msgs.action import ComputePathToPose, FollowPath   # noqa: E402
from rclpy.executors import MultiThreadedExecutor            # noqa: E402

from boids_swarm.world_gen import WorldGenerator             # noqa: E402

BODY_R = 0.15
V_CAP = 3.6
STALL_V, STALL_WIN = 0.3, 2.0


def free_goal(sx, sy, obs, rng_pts=None, min_clear=1.2, min_dist=10.0):
    best, bd = None, -1
    for ix in range(2, 19):
        for iy in range(2, 19):
            x, y = float(ix) + 0.5, float(iy) + 0.5
            if x < 2 or y < 2 or x > 18 or y > 18:
                continue
            if any(math.hypot(x - cx, y - cy) - r < min_clear for cx, cy, r in obs):
                continue
            d = math.hypot(x - sx, y - sy)
            if d > bd:
                best, bd = (x, y), d
    return best


def scenarios(sx, sy):
    out = []
    for name in ('single_block', 'wall_with_gap', 'slalom'):
        c = sp.build_config(name, sx, sy)
        out.append((name, c['obstacles'], [c['goal']]))     # single leg
    of = [tuple(o) for o in WorldGenerator(7, 20.0).generate('obstacle_field').obstacles]
    out.append(('obstacle_field', of, None))      # goals picked on the fly
    # diamond: 4 waypoints, 90-degree corners, an obstacle on each edge and
    # one in the middle -> the path must weave, but never reverses direction
    # (a hand-off that needs a 180-degree pivot would measure wz_max, not
    # the controller).
    out.append(('diamond', [(6.75, 6.75, 1.2), (13.25, 6.75, 1.2),
                            (13.25, 13.25, 1.2), (6.75, 13.25, 1.2),
                            (10.0, 10.0, 2.0)],
                [(3.5, 10.0), (10.0, 3.5), (16.5, 10.0), (10.0, 16.5)]))
    # wall_start: the reactive evader likes to stand against a wall, and a
    # planner start inside the wall's inscribed zone fails. The harness
    # drives the target to y~0.15 (body touching the wall) first.
    out.append(('wall_start', [], [(10.0, 8.0), (16.0, 3.0)]))
    return out


def hug_wall(node, pub, y_target=0.2, timeout=15.0):
    """Raw cmd_vel: face -y, then drive until the body touches the wall."""
    from geometry_msgs.msg import Twist
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        p = node.pose
        if p.y <= y_target:
            break
        err = math.atan2(math.sin(-math.pi / 2 - p.theta),
                         math.cos(-math.pi / 2 - p.theta))
        m = Twist()
        m.angular.z = max(-1.2, min(1.2, 3.0 * err))
        m.linear.x = 1.0 if abs(err) < 0.3 else 0.0
        pub.publish(m)
        time.sleep(0.03)
    pub.publish(Twist())
    time.sleep(0.3)


def _plan(node, goal, obs=()):
    plan = ComputePathToPose.Goal()
    plan.goal = sp._pose_stamped(type(plan.goal), goal[0], goal[1],
                                 node.get_clock().now().to_msg())
    from boids_swarm.behaviors.nav2_evader import plan_start
    p = node.pose
    sx, sy = plan_start((p.x, p.y), (0.0, 0.0), (20.0, 20.0), list(obs), 0.3)
    plan.use_start = True
    plan.start = sp._pose_stamped(type(plan.goal), sx, sy,
                                  node.get_clock().now().to_msg())
    plan.planner_id = 'GridBased'
    f = node.plan_ac.send_goal_async(plan)
    node.wait_for(f.done, 10, 'plan accepted')
    r = f.result().get_result_async()
    node.wait_for(r.done, 10, 'plan result')
    if r.result().status != 4 or len(r.result().result.path.poses) < 2:
        return None
    return r.result().result.path


def drive_chain(node, goals, obs, timeout, handoff=3.0):
    """Follow `goals` in order WITHOUT stopping in between: the next path is
    sent (preempting FollowPath) once within `handoff` m of the current goal,
    which is how Nav2Evader uses the controller (continuous re-goaling).

    Metrics:
      mean_speed_incl_startup  path length / sim time, from the first
                               FollowPath goal to arrival (includes the
                               initial pivot at wz_max and the final stop)
      cruise_speed             mean speed from the first sample with
                               v >= 1 m/s until within 2 m of the final goal
                               (excludes the pivot and the final stop)
    """
    node.samples.clear()
    node.recording = True
    t_sim0 = node.sim_time
    status, gh = None, None
    try:
        for k, g in enumerate(goals):
            path = _plan(node, g, obs)
            if path is None:
                return {'goals': goals, 'reached': False, 'failure': 'plan failed'}
            fg = FollowPath.Goal()
            fg.path, fg.controller_id, fg.goal_checker_id = (
                path, 'FollowPath', 'general_goal_checker')
            ff = node.follow_ac.send_goal_async(fg)
            node.wait_for(ff.done, 10, 'follow accepted')
            gh = ff.result()
            rf = gh.get_result_async()
            last = k == len(goals) - 1
            t0 = time.monotonic()
            while True:
                if rf.done():
                    status = rf.result().status
                    break
                p = node.pose
                if not last and math.hypot(p.x - g[0], p.y - g[1]) < handoff:
                    break
                if time.monotonic() - t0 > timeout:
                    gh.cancel_goal_async()
                    status = 'timeout'
                    break
                time.sleep(0.01)
            if status not in (None, 4):
                break
    finally:
        node.recording = False
    s = list(node.samples)
    final = goals[-1]
    res = {'goals': goals, 'follow_status': status}
    if len(s) < 2:
        res.update(reached=False, failure='no samples')
        return res
    dist = sum(math.hypot(b[2] - a[2], b[3] - a[3]) for a, b in zip(s, s[1:]))
    t = s[-1][1] - t_sim0
    end = math.hypot(s[-1][2] - final[0], s[-1][3] - final[1])
    res['reached'] = bool(status == 4 and end < 0.7)
    res['sim_time_s'] = round(t, 2)
    res['path_travelled_m'] = round(dist, 2)
    res['mean_speed_incl_startup_mps'] = round(dist / t, 2) if t > 0 else 0.0
    i0 = next((i for i, x in enumerate(s) if x[5] >= 1.0), None)
    res['time_to_first_motion_s'] = None if i0 is None else round(s[i0][1] - t_sim0, 2)
    cruise = [x for x in (s[i0:] if i0 is not None else [])
              if math.hypot(x[2] - final[0], x[3] - final[1]) > 2.0]
    res['cruise_speed_mps'] = round(statistics.mean(x[5] for x in cruise), 2) if cruise else None
    res['max_speed_mps'] = round(max(x[5] for x in s), 2)
    clr = min((math.hypot(x[2] - cx, x[3] - cy) - r - BODY_R
               for x in s for cx, cy, r in obs), default=float('inf'))
    wall = min(min(x[2], x[3], 20 - x[2], 20 - x[3]) - BODY_R for x in s)
    res['min_obstacle_clearance_m'] = round(clr, 3)
    res['min_wall_clearance_m'] = round(wall, 3)
    res['collided'] = bool(clr < 0.02 or wall < 0.02)
    # stall: STALL_WIN seconds with mean speed < STALL_V AND not pivoting
    # (< 0.3 rad/s), still > 1 m from the final goal, after first motion.
    stall = False
    i = 0
    for j in range(len(s)):
        while s[j][1] - s[i][1] > STALL_WIN:
            i += 1
        if i0 is not None and i >= i0 and s[j][1] - s[i][1] >= STALL_WIN - 0.2:
            w = [x[5] for x in s[i:j + 1]]
            turn = sum(abs(math.atan2(math.sin(b[4] - a[4]), math.cos(b[4] - a[4])))
                       for a, b in zip(s[i:j], s[i + 1:j + 1])) / (s[j][1] - s[i][1])
            far = math.hypot(s[j][2] - final[0], s[j][3] - final[1]) > 1.0
            if far and statistics.mean(w) < STALL_V and turn < 0.3:
                stall = True
                break
    res['stalled'] = stall
    res['trace'] = [[round(x[1], 2), round(x[2], 2), round(x[3], 2), round(x[5], 2), round(x[4], 2)] for x in s[::6]]
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='nav2_target.yaml')
    ap.add_argument('--label', default='run')
    ap.add_argument('--out', required=True)
    ap.add_argument('--only', default='')
    ap.add_argument('--legs', type=int, default=5, help='goals per obstacle_field chain')
    ap.add_argument('--reps', type=int, default=2, help='chains per scenario')
    ap.add_argument('--timeout', type=float, default=45.0)
    a = ap.parse_args()
    outdir = os.path.dirname(os.path.abspath(a.out))
    os.makedirs(outdir, exist_ok=True)
    rclpy.init()
    node = sp.Spike()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    env = dict(os.environ)
    sim = nav = None
    results = []
    try:
        sim = sp.spawn(['ros2', 'run', 'boids_swarm', 'pygame_sim', '--ros-args',
                        '-p', 'headless:=true', '-p', 'ui_enabled:=false',
                        '-p', 'num_agents:=0', '-p', 'time_scale:=1.0',
                        '-p', 'target_speed_multiplier:=1.8',
                        '-p', 'target_omega_max:=1.2',
                        '-p', f'target_body_radius:={BODY_R}',
                        '-p', 'target_stamina_enabled:=false',
                        '-p', 'episode_time_limit:=0.0', '-p', 'auto_reset:=false'],
                       os.path.join(outdir, f'{a.label}_sim.log'), env)
        node.wait_for(lambda: node.pose is not None, 30, 'sim pose')
        nav = sp.spawn(['ros2', 'launch', 'boids_swarm', 'nav2_target.launch.py',
                        f'config:={a.config}'],
                       os.path.join(outdir, f'{a.label}_nav2.log'), env)
        node.wait_for(lambda: node.lifecycle_active('planner_server')
                      and node.lifecycle_active('controller_server'), 90, 'lifecycle')
        node.wait_for(lambda: node.plan_ac.wait_for_server(0.1)
                      and node.follow_ac.wait_for_server(0.1), 20, 'servers')
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        cli = node.create_client(SetParameters, '/nav2_bridge/set_parameters')
        cli.wait_for_service(5.0)

        from geometry_msgs.msg import Twist
        cmd_pub = node.create_publisher(Twist, '/target/cmd_vel', 10)

        def set_bridge(obs):
            req = SetParameters.Request()
            req.parameters = [Parameter(name='obstacles', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(v) for o in obs for v in o] or [0.0]))]
            fut = cli.call_async(req)
            node.wait_for(fut.done, 5, 'bridge obstacles')

        for name, obs, goals in scenarios(node.pose.x, node.pose.y):
            if a.only and name != a.only:
                continue
            # reposition (empty map) to a spot that is free in the NEXT map
            node.set_sim_obstacles([])
            set_bridge([])
            time.sleep(3.0)
            p0 = node.pose
            spot = (goals[0] if name == 'diamond'
                    else (10.0, 2.0) if name == 'wall_start'
                    else free_goal(p0.x, p0.y, obs, min_clear=1.5))
            try:
                drive_chain(node, [spot], [], a.timeout)
            except Exception as e:           # noqa: BLE001
                print('reposition failed', e, flush=True)
            node.set_sim_obstacles(obs)
            set_bridge(obs)
            time.sleep(3.0)                  # map -> costmaps
            if name == 'wall_start':
                hug_wall(node, cmd_pub)
            chains = []
            goals_fixed = list(goals) if goals else []
            for rep in range(a.reps):
                p = node.pose
                if name == 'obstacle_field':
                    goals, q = [], (p.x, p.y)
                    for _ in range(a.legs):
                        q = free_goal(q[0], q[1], obs)
                        goals.append(q)
                elif name == 'wall_start':
                    if rep > 0:
                        hug_wall(node, cmd_pub) if node.pose.y > 0.3 else None
                    goals = list(goals_fixed)
                elif name == 'diamond':
                    ring = goals_fixed
                    i0 = min(range(4), key=lambda i: math.hypot(
                        ring[i][0] - p.x, ring[i][1] - p.y))
                    goals = [ring[(i0 + 1 + k) % 4] for k in range(5)]
                else:
                    g1 = sp.pick_goal(p.x, p.y)
                    if any(math.hypot(g1[0] - cx, g1[1] - cy) < r + 1.0 for cx, cy, r in obs):
                        g1 = free_goal(p.x, p.y, obs)
                    goals = [g1]
                try:
                    c = drive_chain(node, goals, obs, a.timeout)
                except Exception as e:      # noqa: BLE001
                    c = {'reached': False, 'failure': repr(e), 'goals': goals}
                chains.append(c)
                print(name, rep, json.dumps({k: v for k, v in c.items() if k != 'trace'}), flush=True)
                time.sleep(0.5)
            results.append({'scenario': name, 'legs': chains})
    finally:
        sp.stop(nav)
        sp.stop(sim)
    allv = [l for r in results for l in r['legs'] if 'sim_time_s' in l]
    cr = [l['cruise_speed_mps'] for l in allv if l.get('cruise_speed_mps')]
    tot_d = sum(l['path_travelled_m'] for l in allv)
    tot_t = sum(l['sim_time_s'] for l in allv)
    summary = {
        'label': a.label, 'config': a.config, 'v_cap': V_CAP,
        'chains': len(allv),
        'all_reached': all(l.get('reached') for r in results for l in r['legs']),
        'any_collision': any(l.get('collided') for l in allv),
        'any_stall': any(l.get('stalled') for l in allv),
        'cruise_speed_mean_mps': round(statistics.mean(cr), 2) if cr else 0.0,
        'cruise_speed_min_mps': min(cr, default=0.0),
        'mean_speed_incl_startup_mps': round(tot_d / tot_t, 2) if tot_t else 0.0,
        'results': results}
    with open(a.out, 'w') as f:
        json.dump(summary, f, indent=2)
    print('SUMMARY', json.dumps({k: v for k, v in summary.items() if k != 'results'}))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
