# Relay E1b: post-hoc follow-up diagnostic (2026-10-08)

**Status: NOT pre-registered.** This was designed after the E1 results, in response to red-team review, to answer two questions that E1 left open. It is a small diagnostic, not a hypothesis test; read the numbers as descriptions of 8 or 4 runs per cell. E1 itself (`artifacts/relay-e1-2026-10-08/`) is unchanged.

Questions:
1. Does QoS depth 10 only turn drops into **delay or timeout rejections** (the sighting arrives late and the receiver discards it)?
2. Does the drop rate measured under **fast-forward** (headless, sim `time_scale` 4) differ from **real time** (`time_scale` 1)?

## Setup
- Scene S12 (12 agents), intercept, env open, perception sensor, capture hull, 30 s, `comm_range = radio_range = 8.0`, all as in E1; modes ros-d1 and ros-d10 only.
- Fast-forward cells (`S12ff`): seeds 101-108, time_scale = simulator default 4.0. Real-time cells (`S12rt`): seeds 101-104, `time_scale:=1` (new launch argument). Seeds 101-108 do not overlap E1 (1-20). 24 runs, strictly sequential, shuffled order (`shuffle_seed` 20261009), ROS_DOMAIN_ID 201, CPU only. **0 invalid runs.** Matrix: `tools/relay_e1b_matrix.json`; runner `tools/run_relay_e1.py` (extended to subscribe to `/swarm/relay_events`, RELIABLE, depth 100000, saved per run); analysis `tools/analyze_relay_e1b.py`. Raw runs are in `runs/` (untracked); tracked: this README and `summary.json`. Figure: `docs/media/relay_e1b_age.png`.
- `/swarm/relay_events` already carried `age` (= controller clock at receive minus the sighting's observation stamp, sim time), `reason` and `receiver_id`; no controller change was needed. Controller clocks follow `/clock`, which ticks at 30 Hz, so **age is quantised to 1/30 s and any delay under about 33 ms reads as 0**.
- Only events a controller callback actually received are logged. A sighting dropped by DDS before the callback never appears; that loss is measured through the drop rate (same definition and receiver counters as E1), not through events.
- Age statistics use delivered non-self events except `awaiting_episode` (clock not yet valid); reason shares use all delivered non-self events (`all`) or only those that were in range (`in-range`: excludes `out_of_range`, `awaiting_episode`).

## Results
Drop = per-receiver in-range drop, pooled, same definition as E1 (including its boundary-mismatch caveat). The 95% interval is a run-level bootstrap (10,000 resamples, seed 20261011); with 8 or 4 runs per cell the intervals are rough.

| cell | runs | captures | drop pooled [95% CI] | age median | age p95 | age p99 | age max | share age > 0.6 s | accepted (in-range) | stale (in-range) | late_observation (in-range) | other (in-range) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ff ros-d1 | 8 | 4 | 55.8% [53.4, 57.5] | 0 s | 0.033 s | 0.100 s | 0.333 s | 0 | 79.8% | 0 | 19.3% | 0.9% |
| ff ros-d10 | 8 | 4 | 7.7% [4.3, 11.1] | 0 s | 0.067 s | 0.150 s | 0.633 s | 0.05% (19 of 40,501) | 69.1% | 0.05% | 29.6% | 1.2% |
| rt ros-d1 | 4 | 2 | 52.3% [50.5, 53.9] | 0 s | 0.017 s | 0.050 s | 0.383 s | 0 | 84.6% | 0 | 15.1% | 0.4% |
| rt ros-d10 | 4 | 2 | 0.3% [-0.05, 5.4] | 0 s | 0.017 s | 0.033 s | 0.050 s | 0 | 82.1% | 0 | 17.8% | 0.04% |

Share of all delivered non-self events (`all`): out_of_range 14.1% / 14.4% / 12.3% / 12.7% (ff d1, ff d10, rt d1, rt d10), consistent across cells because it only reflects geometry. Remaining `other` reasons were `awaiting_episode` (start of the episode), `receiver_pose_time_mismatch` (ff d1 177, ff d10 342, rt d1 52, rt d10 0), `receiver_pose_unavailable` and `future_stamp` (a handful). Per-cell numbers, per-run drop rates and per-run age p95 ranges are in `summary.json`.

Receivers with no status message (excluded from drop numerator and denominator): ff d1 10, ff d10 10, rt d1 5, rt d10 0.

## What this shows
**Q1: depth 10 does not convert drops into timeout rejections (in this data).**
- `stale` (age above `sighting_timeout` 0.6 s) was 19 of 40,501 delivered events (0.05%) for ff ros-d10 and zero in the other three cells; it is not where the recovered deliveries go.
- Depth 10 does lengthen the age tail somewhat under fast-forward: p95 0.067 s vs 0.033 s, p99 0.150 s vs 0.100 s, max 0.63 s vs 0.33 s. In real time the tails were the same or shorter (p95 0.017 s in both). The fast-forward tail difference is small relative to the 0.6 s timeout.
- The much more visible effect is `late_observation`: 29.6% of in-range events in ff d10 vs 19.3% in ff d1 (17.8% vs 15.1% in real time). This reason means the observation was not newer than one the controller had already applied, i.e. it was superseded, not delayed. Inference: with the lost half restored, a larger share of the delivered stream is redundant (several agents report the same target and only the newest stamp is used), which would be one reason why restoring deliveries did not visibly change capture in E1. This is not tested here.

**Q2: fast-forward vs real time.**
- Depth-1 loss exists in real time too: 52.3% [50.5, 53.9] real time vs 55.8% [53.4, 57.5] fast-forward. The QoS-depth effect on delivery is therefore not an artifact of fast-forward. The 3.5 pp difference between ff and rt is small and the intervals only just fail to overlap.
- Depth-10 loss: real time 0.3% (one of four runs at 7.3%, hence the wide upper bound 5.4%) vs fast-forward 7.7% [4.3, 11.1] (per run -0.8% to 18.7%). That is higher than E1's 1.8% for S12 ros-d10, so the depth-10 loss is not always about zero.
- Confound (important): these runs shared the machine with unrelated jobs and the load average before runs ranged from 13 to 68 (E1: 14-24), and the simulator could not keep up with `time_scale` 4. For uncaptured 30 s runs the wall times were 15-40 s for fast-forward (a true 4x would take about 10 s including launch) and 32-48 s for real time (about 33 s expected; the 48 s run fell behind real time). So "fast-forward" here is effectively a variable 1-2.6x and the ff vs rt contrast is a mix of time scale and load. Inference: the higher d10 loss in the ff cells is consistent with controllers falling behind under load (more queue overflow even at depth 10, more `receiver_pose_time_mismatch`), but this was not isolated.

## Deviations and caveats
1. Not pre-registered; sample sizes (8, 4) were fixed by the follow-up brief before the runs; no run was dropped or repeated. Capture counts (4/8, 4/8, 2/4, 2/4) are shown only for completeness and carry no inference.
2. **Source fingerprint differs between runs (5 values), and two git commits are recorded (`849d45a`, `c62102d`).** I committed analysis/tooling changes while the matrix was running. Between the two commits only `relay_stats.py`, `experiment_matrix.py`, `run_relay_e1.py` (already loaded in the running process), `analyze_relay_e1.py` and tests changed (`git diff 849d45a c62102d --stat`); no runtime file (comms, controller, simulator, launch) changed, so all 24 runs executed the same runtime code. Run configs also show `working_tree_dirty` for the same reason.
3. Age is sim-time and 1/30 s quantised; delays shorter than one tick are invisible, and the callback logs only what DDS delivered.
4. The drop definition inherits E1's caveats (boundary mismatch, so d10 values within about +/-1 pp of zero are "approximately zero"; receivers without status excluded).
5. Machine load was high and variable (see Q2); results about d10 loss and the ff/rt comparison should be treated as indicative only.
