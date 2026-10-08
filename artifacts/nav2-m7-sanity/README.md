# Nav2 evader sanity benchmark (M7) — rerun after the red-team fixes

**This is a sanity check, not a pre-registered experiment.** Six seeds per
evader, two repeats of the whole sweep, on a loaded shared machine. It shows
that the full pipeline (`evader:=nav2` launch -> Nav2 -> blend -> sim) runs
and records its failure modes. **No claim is made that nav2 is stronger or
weaker than reactive**: the two repeats of the same code below disagree with
each other by more than any difference between the evaders.

## Setup

`tools/nav2_m7_sanity.py --seeds 1,2,3,4,5,6` (all six seeds, none skipped),
summary `summary.json` (repeat 2, the one with per-mode speeds) and
`run1_summary_without_speed_by_mode.json` (repeat 1, same code, harness
without the speed breakdown). The pre-fix results (5 seeds, old footprint)
are kept as `before_red_team_summary.json` and
`earlier_run_global_radius_0.30_summary.json`.

- 12 agents, `strategy:=intercept`, `env:=obstacle_field` (fixed maze),
  `capture_mode:=hull`, shipped balance (target 3.6 m/s, omega 1.2, stamina on),
  `time_scale:=1.0`, `warmup:=8`, `pursuer_delay:=4`; the harness counts a 30 s
  window from the first boid movement.
- Seed 1 is included this time. (The old README said seed 1 was captured in
  ~2 s by either evader; that was not reproduced: reactive seed 1 is captured
  at ~12 s in both repeats. The earlier observation was probably contaminated
  by stray processes on the same ROS domain; unverified.)
- **Measures changed** (the old ones were misleading): mode shares are
  **time-weighted** (each `/target/evader_status` message's mode held until the
  next, inside the window) instead of per-tick counts; the plan failure rate is
  `plan_fail / plan_requests` and `FollowPath` aborts are reported **separately**
  (`follow_abort / follow_started`), not mixed into one "failure" number.
  "Speed" is the mean of the target's `/target/pose` speed over the window, i.e.
  the **in-game speed**, which is capped at 2.0 m/s (swarm cruise speed) unless a
  boid is within 6 m (3.6 m/s sprint, stamina permitting). It is not the
  tuning harness's controller-capability speed.

## Repeat 2 (summary.json; per-mode speeds recorded)

| seed | reactive | nav2 | nav2 time share nav2/blend/reactive | nav2 plan fail / requests | follow aborts / started | nav2 speed in nav2 / blend / reactive mode (m/s) |
|---|---|---|---|---|---|---|
| 1 | captured 11.95 s, 2.66 m/s | captured 10.80 s, 1.98 | 0.08 / 0.39 / 0.53 | 2 / 10 | 1 / 8 | 0.00 / 1.33 / 2.73 |
| 2 | timeout, 2.95 | timeout, 2.36 | 0.05 / 0.55 / 0.40 | 1 / 26 | 1 / 25 | 0.92 / 2.26 / 2.70 |
| 3 | timeout, 2.85 | captured 8.20 s, 1.92 | 0.16 / 0.57 / 0.28 | 0 / 12 | 0 / 12 | 0.20 / 2.11 / 2.48 |
| 4 | timeout, 2.87 | captured 12.25 s, 2.02 | 0.14 / 0.23 / 0.63 | 2 / 17 | 1 / 15 | 0.15 / 1.45 / 2.63 |
| 5 | captured 10.38 s, 2.70 | captured 18.42 s, 2.40 | 0.00 / 0.48 / 0.52 | 0 / 13 | 1 / 13 | - / 1.96 / 2.80 |
| 6 | timeout, 2.86 | timeout, 2.87 | 0.09 / 0.02 / 0.89 | **15 / 22** | 1 / 7 | 1.28 / 0.77 / 3.07 |
| all | 2/6 captured, mean capture 11.2 s, mean speed 2.81 | 4/6 captured, mean 12.4 s, mean speed 2.26 | 0.09 / 0.37 / 0.54 | **20 / 100 = 20 %** | **5 / 80** | |

## Repeat 1 (same code, no per-mode speed)

| seed | reactive | nav2 | nav2 time share nav2/blend/reactive | plan fail / requests |
|---|---|---|---|---|
| 1 | captured 12.34 s, 2.64 | captured at sim 9.8 s, i.e. **before the chase started** (3 attempts, window undefined, excluded from time/speed stats) | - | - |
| 2 | captured 14.84 s, 2.64 | captured 6.50 s, 1.18 | 0.32 / 0.18 / 0.50 | 0 / 9 |
| 3 | captured 7.58 s, 2.69 | captured 5.30 s, 1.49 | 0.43 / 0.39 / 0.18 | 0 / 8 |
| 4 | captured 5.86 s, 2.64 | captured 11.70 s, 1.83 | 0.14 / 0.45 / 0.41 | 1 / 13 |
| 5 | timeout, 2.83 | captured 18.08 s, 2.55 | 0.04 / 0.55 / 0.41 | 0 / 17 |
| 6 | timeout, 2.87 | captured 15.36 s, 2.37 | 0.16 / 0.24 / 0.61 | 2 / 14 (+1 abort) |
| all | 4/6 captured, mean 10.1 s, speed 2.72 | 6/6 captured (5 with a defined window), mean 11.4 s, speed 1.88 | 0.22 / 0.36 / 0.42 | **3 / 61 = 4.9 %**, follow aborts 1 / 58 |

Across repeats the reactive capture count went 4/6 -> 2/6 and nav2's 6/6 ->
4/6 for identical code. That noise is the reason no ranking is claimed.

## Compared with the pre-fix sanity run (5 seeds, 2,3,4,5,6)

- Plan failures: **45 of 103 requests (44 %) before; 3 / 61 (5 %) and 20 / 100
  (20 %) in the two repeats.** (The old "48 of 103" mixed 45 plan failures with
  3 follow aborts; the correct split was 45 plan failures + 3 aborts of 103.)
  The pocket failures from (3.2, 0.3) are gone in repeat 1 and mostly gone in
  repeat 2.
- Mode shares: old 0.22 / 0.33 / 0.45 were per-tick; time-weighted they are
  0.22 / 0.36 / 0.42 (repeat 1) and 0.09 / 0.37 / 0.54 (repeat 2). In both the
  target is in **pure nav2 mode only 9-22 % of the time**.

## What is still wrong (observed, not fixed)

1. **Seed 6, repeat 2: 15 plan failures, all from the same start (2.95, 0.15)**
   over ~23 s: the target stood against the bottom wall in the pocket and
   never left. A live check (`obstacles of seed 6`, no boids) shows the
   corrected costmap prices that cell at 203 (traversable) and plans from it
   succeed to the same goals, so the failure needs boids: inference — a boid
   at the pocket mouth (the only exit) blocks or inflates the way out, i.e.
   the target **did trap itself in the pocket**. The goal scorer's pocket
   term does not stop the *reactive* term from walking the target in there.
2. **In-game Nav2 speed is low**: 1.9-2.4 m/s average (reactive 2.7-2.9), and the
   speed while in pure `nav2` mode is 0-1.3 m/s (pivots at 1.2 rad/s after each
   new goal preempts the path, plus the 2.0 m/s non-panic cap). Reading: Nav2
   mode is mostly a hand-over state before blending, not a fast escape.
3. Nav2 itself still does not react to boids (collision detection is off); the
   2 Hz costmap + plan period is slower than the boids.
4. Repeat 1 seed 1: the Nav2 target walked into the still-stationary swarm
   before the chase (captured at 9.8 s), a startup artefact of spawning in
   the swarm, which the harness correctly discarded.
5. Single machine, load average 5-10 from other jobs; the sim is wall-clock
   coupled at `time_scale:=1.0`.

Code state: commit `4f918aa` plus the following commit (sanity harness with
time-weighted modes and per-mode speeds; docs).
