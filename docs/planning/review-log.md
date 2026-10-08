# Review log

Technical rulings from the adversarial review rounds ("red" = attack the claim, "white" = check for over-optimism) held while building this project. Each finding is recorded with the ruling and the reason. Dates are 2026-10-08.

## Scope decided before implementation

- Two work items: (1) the next phase of the sighting relay (diagnose zero captures, then capture and performance numbers, then charts and a demo); (2) the SDD v3 M7 Nav2 evader.
- Unverified phase 1 relay code was verified first (pure tests plus ROS integration tests), then committed on its own branch, and the older artifacts were marked void.
- Inventory: v3 M0-M6 done; M7 Nav2 existed only as a stub; v4 M8-M14 done. The phase 1 artifacts had 12 timeouts and zero captures, and an older artifact folder contradicted the final numbers.

## Red round 1 (relay phase 1)

| Finding | Ruling | Reason / action |
|---|---|---|
| F1 range mismatch | Upheld | `comm_range` 12 (multi-hop DSU) vs `radio_range` 8 (single hop) in params.yaml / comms.py. Comparisons must align range and hops. |
| F2 zero captures, no baseline | Upheld | Find a scenario where oracle captures, then compare modes. |
| F3 small sample, same seed not reproducible | Upheld | At least 5 seeds per cell, confidence intervals, no bitwise-determinism claim. |
| M1 one sighting per tick | Partly upheld | Direct-over-relay priority is reasonable but needs an ablation. |
| M2 "distributed" wording overstated | Upheld | Reworded to "ROS one-hop transport with an application-level range gate". |
| M3 shared sightings BEST_EFFORT depth=1 | Upheld, to be verified | Depth ablation added (later experiment E1). |
| m2 uncommitted work | Resolved | Committed on a feature branch. |

## White round 1

Adopted:
- Ship a visible artifact first (README GIF) and fix the "truly distributed" wording immediately, with limitations stated.
- Time-boxed Nav2 spike on a separate branch to measure startup time, planning latency and obstacle avoidance before full commitment; if it fails, degrade and report honestly.
- A small diagnostic instead of a large sweep (align range to 8 m, oracle vs ros, 5 runs each; measure drop rate, oracle capture rate, wall-clock cost). Then **pre-register** scenario and criteria and do not change the scenario if results disappoint.
- One integrated story and one baseline table: {reactive, nav2} evader x {off, oracle, ros} relay.
- Report raw counts plus Wilson intervals, and state runs are not bitwise reproducible.
- Merge the SDDs into one short addendum rather than two full SDDs.

## Red round 2 (E1 results)

| Finding | Ruling | Reason / action |
|---|---|---|
| Published data mismatched raw data | Not upheld | Cross-checked 509 agent-runs against published row counts; differences were 0 or +1. H1 recomputed identically and per-run ranges of d1 and d10 do not overlap, so H1 is robust. |
| "Capture time 7 s vs 14 s" is conditional on capture | Upheld | Replaced by a cumulative capture curve (Kaplan-Meier, 30 s censoring), labelled exploratory and not pre-registered. No "twice as fast" claim, no cherry-picked 10 s threshold p-value. |
| d10 may turn loss into delay; sighting age and rejection reasons not measured | Upheld | Added a follow-up diagnostic, clearly marked non-registered. |
| Fast-forward drop rate may not represent real time | Upheld | Added a small real-time (`time_scale` 1) sample (E1b). |
| "Delivery loss isn't what limits capture" overstated | Upheld | Downgraded to "not detectable at n=20; about 80+ runs per cell needed". |
| Deviations list imprecise | Upheld | Split into "implementation choices the pre-registration did not cover" and "true deviations". |
| No run-level interval for drop rate | Upheld | Added run-level bootstrap CI. |
| `<=` vs `<` range boundary inconsistent in comms.py | Upheld (minor) | Unified and tested. |
| Track-valid denominator varies with episode length | Partly upheld | Report a fixed window or state the limitation. |

## Red round 3 (Nav2 M7)

Checked against nav2_target.yaml (`robot_radius` 0.22), occupancy.py (`border_cells` 2), `start_margin` 0.3 and plan-failure counts in the log (3 / 25 / 2 / 15).

| Finding | Ruling | Reason / action |
|---|---|---|
| Self-trapping not solved; pocket failures come from inconsistent parameters | Upheld (critical) | Make footprint, border and start margin consistent; blacklist failed goals; no retry during fallback; score escape points by reachability (grid geodesic distance). |
| Name vs reality: pure Nav2 only about 22% of the time | Upheld | Re-measure with time-weighted mode share; if still reactive-dominated, say so. |
| "48/103" mixes denominators | Upheld | Report plan-failure rate and follow aborts separately. |
| Collision detection off makes the local costmap inert | Upheld | Retry enabling it once the footprint is aligned. |
| Linear (v, w) blending can cancel | Upheld | Blend in heading space or add a reactive safety override. |
| Weak integration tests (cost>0 without negative control, no fallback test, 198 was a collected count) | Upheld | Tests strengthened. |
| "All 7 MPPI configs failed" overstated | Upheld | Reworded to "4 valid configs"; the radius fix was not applied to MPPI. |
| Speed definitions loose | Upheld | Documents use in-game speed. |
| Nav2 needs `time_scale` 1 | Upheld | Evader experiments cost real time. |

## White round 2

- **Cancel the E2 capture-rate comparison** (n=40 almost certainly cannot separate the groups). Replace with mechanistic M7 acceptance metrics: self-trap count, plan-failure rate, time-weighted Nav2 share, obstacle-avoidance success.
- **Stop-loss for Nav2:** after the fixes and re-measurement, if the Nav2 share is still low, state that Nav2 is a planning sub-module with the reactive controller taking over at close range. No further tuning; MPPI is not revisited.
- Next steps: merge branches, repo-relative quickstart, artifacts index, README rewrite (<=120 lines, GIF first, one-line conclusion, honest limitations, old content moved to docs/), privacy audit, one more red review of the README. No red/white round after every step; keep the decision record short.

## Control panel, smart evader and dominance selector (review notes)

Dates are 2026-10-08. All numbers below come from non-registered sanity runs; none is a hypothesis test.

| Question | Ruling | Reason / action |
|---|---|---|
| Is the evader degrading when it runs into walls? | Not a regression | `ReactiveEvader` steers by local repulsion and has no wall-escape logic by design; wall contact is its normal behaviour. The `smart` evader was added as a separate brain instead of changing the reactive one, so registered runs keep their behaviour. |
| Why did the evader look "dumb" in Sensor + ROS mode? | Not the sensing mode | Baseline and Sensor + ROS drive the same evader and behave the same. The real cause was the spawn rule: `_free_pos` draws 200 random positions and falls back to the arena centre when none is far enough from the boids, which happened for about half the seeds with 12 boids, so the target started surrounded and was caught in about 1 s. Fix: `spawn.py` (`spawn_safe`, `spawn_min_clearance`) plus `capture_grace`, on by default in the window only. Headless and pre-registered runs keep the original rule. |
| Keep the dominance-region selector for `smart`? | Keep, with the limit stated | It improves survival in `open` and, for Nav2, in-mode speed (0.31 to 2.53 m/s) and plan failures (24/49 to 3/43, n=5). In `obstacle_field` with 12 pursuers there is no visible difference (all caught in 3-7 s), so no claim of a stronger evader there. `selector: sampled` stays available for comparison. |
| Why did the Nav2 target turn on the spot? | Controller behaviour, not preemption | The RegulatedPurePursuit controller rotates in place when the look-ahead point is more than `rotate_to_heading_min_angle` off the heading, which happens after any goal change pointing backwards. Goal preemption was ruled out. Fix: `smooth_start` replaces the head of a new path with a turning arc so the follower turns while moving; the planner footprint radius and a wall penalty on goals were also adjusted. |
| Can the panel results be compared with the pre-registered experiment? | No | Window runs use `spawn_safe` and `capture_grace`; the registered runs use the original spawn rule. The README states this. |
| Also fixed | | Launched in the background, `kill -INT` was ignored (inherited SIGINT disposition); `ensure_sigint_deliverable` restores it. Agents are drawn larger so they are visible at normal window sizes. |

