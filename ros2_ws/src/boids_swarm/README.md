# boids_swarm

ROS 2 Boids Swarm — **Cooperative Pursuit Game** (pygame display + ROS 2 core).
Implementation of [SDD v3](../../../SDD/Sdd_boids_pursuit_v3.md).

A swarm of `N` boids must catch a target that moves at **2× swarm speed** —
impossible for any single agent, so cooperation (prediction, flanking,
encirclement, herding) is the only winning strategy. The target is fast but
turns badly (`target_omega_max` is the balance knob).

## Architecture (SDD §1)

| Node | Count | Role |
|---|---|---|
| `pygame_sim` | 1 | **The world**: physics, collisions, capture, rendering, HUD, pose/`/clock` publishing |
| `boid_controller` | N | Per-agent flocking + pursuit (namespace `/agent0..N-1`) |
| `target_controller` | 1 | The evader (`ReactiveEvader`; `Nav2Evader` = M7 stretch) |

Controllers talk to the world only via topics: `/swarm/poses`
(aggregated, one subscription per controller — the v2 §10.2 scaling fix),
`/agentI/pose` (debug), `/target/pose`, `/agentI/cmd_vel`, `/target/cmd_vel`.
The sim publishes `/clock`; controllers run with `use_sim_time` so headless
fast-forward benchmarking stays fair.

## Build & run

```bash
cd ~/final_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select boids_swarm
source install/setup.bash

# M2 — pure flocking, no target
ros2 launch boids_swarm flocking.launch.py num_agents:=12

# M3+ — the pursuit game (AI evader)
ros2 launch boids_swarm pursuit.launch.py num_agents:=12 strategy:=encircle trails:=true

# M6 — drive the 2x target yourself with arrow keys
ros2 launch boids_swarm pursuit.launch.py game_mode:=human strategy:=herd

# obstacles: "x,y,r" circles separated by ';'
ros2 launch boids_swarm pursuit.launch.py obstacles:="6,6,1.5;14,12,2"

# headless benchmark (bounded fast-forward, deterministic dt)
ros2 launch boids_swarm pursuit.launch.py headless:=true episodes_max:=6 \
    time_limit:=90.0 strategy:=naive seed:=11
```

## Pursuit strategy ladder (SDD §4.4) — `strategy:=`

| | Strategy | Idea |
|---|---|---|
| T0 | `naive` | aim at target's current position — control experiment, **loses** vs 2× |
| T1 | `intercept` | lead pursuit: aim at predicted future position |
| T2 | `pincer` | boids ahead of target loop far ahead; boids behind chase |
| T3 | `encircle` | claim angular ring slots, ring shrinks over time |
| T4 | `herd` | attack from the open side only → drive target into a corner → collapse ring |

Capture (SDD §4.5): `capture_mode:=hull` (default; target inside convex hull
of ≥`capture_k` boids within `d_capture`) | `escape_blocked` | `tag` (HP).

## Live tuning

Every parameter is runtime-reconfigurable:

```bash
ros2 param set /agent3/boid_controller pursuit_strategy herd
ros2 param set /pygame_sim render_trails true
ros2 param set /pygame_sim screenshot_dir /tmp/shots   # periodic PNGs
```

Defaults: [config/params.yaml](config/params.yaml).

## Tests

```bash
python3 -m pytest src/boids_swarm/test/ -q   # 90 pure-math tests (v3 + v4)
```

## Hard-won implementation notes (read before touching the control loop)

1. **Never rely on one `rclpy.spin_once()` per frame.** It processes ONE
   callback. Against 13 cmd_vel streams the QoS queues fill ⇒ ~330 ms
   actuation delay ⇒ every steering loop oscillates and flocking dies
   (and per-frame drain loops burn the frame budget instead). The sim
   runs the executor in a background thread — callbacks only do atomic
   assignments, pygame stays in the main thread — and cmd_vel subs use
   queue depth 1 (only the latest command matters).
2. **O(N²) pose subscriptions saturate Python.** N controllers × N pose
   topics at 60 Hz pegged 12 cores. Fixed with the aggregated
   `/swarm/poses` array (SDD v2 §10.2 option) at 30 Hz.
3. **±π steering chatter.** With sensing latency, a P heading controller
   flip-flops full-left/full-right when the goal is directly behind
   (the sign of the wrapped error jitters). `to_twist` now has turn-direction
   hysteresis (`last_w`) beyond |e| > 2.6 rad.
4. **Deadband must not freeze headings.** At flock equilibrium |V|≈0 and
   `atan2` is noise. Below `v_deadband` boids stop translating but still
   rotate toward the local mean heading — otherwise alignment consensus
   never forms.
5. **Desired-vector EMA** (`v_filter_alpha`) + own-heading extrapolation by
   published ω compensate the pose-cache latency.

## Game balance (SDD §4.2) — measured

An untiring 2× target in an open arena is uncatchable by construction —
it cruises just outside `panic_distance` forever. The shipped balance uses
the SDD's own knobs:

- `target_omega_max: 1.2` (turn radius ≈ 3.3 at sprint — "fast but turns badly")
- `target_stamina_enabled: true` — the evader sprints only when a pursuer
  is within `panic_distance`, so the swarm gets closing windows
- `encircle` fans out with a distance-scaled ring (surround-then-squeeze)
  instead of tail-chasing as one clump

Benchmark (seeded, 6–4 episodes, 90–180 s limits):
`naive` ≈ 1/6 (spawn luck only, as the SDD predicts for T0);
`intercept` 2/6; `herd` 2/6; `encircle` 2/6 @90 s and **3/4 @180 s** —
possible-but-hard, per spec. Long open-field stalemates still occur;
`w_pursuit`, `panic_distance`, stamina drain/regen and `d_capture` are the
knobs to iterate live (`ros2 param set`).

## v4 — perception, advanced control, procedural worlds (M8–M14)

v4 addresses the two v3 playtest findings: the untiring 2× target survives
by hugging the perimeter (a mathematically-safe 1D loop), and perception
was unrealistic (global ground truth for everyone). New modules:
`perception.py`, `tracking.py`, `comms.py`, `world_gen.py`; new strategies
in `behaviors/pursuit.py`; adaptive target in `behaviors/evasion.py`.

**Perception model (`perception_mode:=sensor`).** The sim synthesizes each
agent's own detections — FOV cone, occlusion ray-casts, distance-growing
noise, dropout — and publishes only `/agent{i}/detections` (relative
range/bearing). `perfect` keeps the v3 broadcast as a regression baseline.

```bash
# sensor perception, narrow FOV, information sharing on
ros2 launch boids_swarm pursuit.launch.py perception:=sensor strategy:=encircle
```

**Tracking + search + comms.** Each controller runs an alpha-beta track
filter with data association (`tracking.py`), derives heading/speed from
track velocity, searches (coverage sweep) when the target is lost, and —
via the range-limited mesh (`comms.py`) — shares sightings so the *whole
swarm* tracks a target most individuals can't see.

**New tactics** (`strategy:=`): `counter_rotate`, `blockade`, `corner_trap`,
`herd_inward` (M11 anti-perimeter, on a shared circling detector); `sweep`
(wall-anchored cordon), `role_encircle` (asymmetric collapse), `bait`
(M12).

**`auto` (the default): decentralized strategy selection.** Every boid
picks its own tactic each cycle from the shared belief — `blockade` when
the target circles the perimeter, `corner_trap` when it's pinned near a
corner, `encircle` in the open — so the swarm adapts with no coordinator.

**Terminal commit (kills the orbit).** Ring/lead tactics offset their aim
from the target, so a boid that gets close used to *circle* it instead of
closing. Inside `commit_distance` the aim blends to a straight pounce, so
the net collapses decisively. Measured: `auto` **5/9** captures at avg
**3.7–6 s** vs `encircle`'s 3/9 and slower — faster and more reliable.

```bash
ros2 launch boids_swarm pursuit.launch.py strategy:=auto env:=obstacle_field stamina:=true
```

**Procedural environments** (`env:=`): `obstacle_field` (a **fixed,
hand-authored maze** — 16 varied-size circles spread across the arena with
a clear passage between every pair, identical every run; obstacle
avoidance also slides tangentially around a circle so agents arc past
smoothly), `pillar`, `zones` (capture zone = win, tar pit = nullify 2×
speed; seed-derived), `shrink` (battle-royale bounds contraction — kills
the perimeter loop).

```bash
ros2 launch boids_swarm pursuit.launch.py env:=shrink strategy:=encircle stamina:=true
ros2 launch boids_swarm pursuit.launch.py env:=zones strategy:=herd_inward
```

**Adaptive target** (`evader:=adaptive`): a utility-based selector over a
behavior repertoire (retreat / perimeter-run / juke / gap-dash / obstacle-
shield) with hysteresis — logs mode switches tied to threat geometry.

### v4 verification (measured)

| Milestone | Result |
|---|---|
| M8 sensor | FOV gates detections (omni K≈5–7 → narrow-cone K=0–3); perfect reproduces v3 |
| M9 tracking | flocking stable under noise+dropout (min-dist 0.77 vs 0.55, hvar 0.22 vs 0.12); search fans the swarm out and reacquires |
| M10 comms | belief spread **2.3/12 → 12/12** agents when comms on (the individual-vs-swarm gap) |
| M11 tactics | **blockade 7/9** vs encircle 3/9 on the same seeds (perimeter-hugging target) |
| M12 formations | sweep/role_encircle/bait run in-sim; bait captured 1/1 @6.2s |
| M13 worlds | zones render + capture-zone win + tar-pit slow; shrink squeezes to a tiny box, capture 3/4 |
| M14 adaptive | switches retreat/perimeter/juke/gap_dash by threat geometry (logged) |

Everything is gated behind toggles (`perception_mode`, `env`, `evader`,
`comms_enabled`, per-sensor ablations) so v3 benchmarks stay comparable.

## Milestones (SDD v3 §7.3)

M0–M6 implemented and verified (M2 metrics: min pairwise ≈ 0.7–1.4, heading
variance → 0.03–0.08, bbox 184→17; M3+: episodes/score/HUD live; strategy
A/B via seeded headless benchmarks; M6: obstacles render/block, human mode
drives the target with arrow keys). M7 (`Nav2Evader`) is a stub with the
§6.4 plumbing contract documented — Nav2 packages are installed and the sim
already owns the world state needed to publish odom/TF/map (the v4 adaptive
target's retreat-to-open behavior is the job that finally justifies it).

## Tests

90 pure-math unit tests (`pytest src/boids_swarm/test/`): flocking, capture
geometry, pursuit strategies (v3 + M11 + M12), perception (FOV/occlusion/
noise/dropout/round-trip), tracking (alpha-beta/association/circling), comms
mesh, world generation, adaptive evader.
