# Sighting relay diagnostic (2026-10-08, branch feat/portfolio-phase2)

Small CPU-only diagnostic for pre-registering phase 2. Not a performance comparison. All numbers below are from the raw JSONL in this folder (untracked by git except this README and summary.json).

## Setup
- Scenario: intercept, ai target, perception=sensor, capture_mode=hull, env=open, episodes_max=1, time_limit=30 s, `comm_range=8.0`, `radio_range=8.0` (params.yaml defaults are 12.0 / 8.0).
- `comm_range:=` is NOT a launch argument. Range override is done by `ros2_ws/src/boids_swarm/tools/diag_pursuit.launch.py` (new file): runs the unmodified `pursuit.launch.py` with a temp copy of params.yaml where only those two values are replaced (env `DIAG_COMM_RANGE`, `DIAG_RADIO_RANGE`). No file under ros2_ws/src was modified; the 40 hashed source files are byte-identical to `artifacts/sighting-relay-final-2026-10-07/source_snapshot.tar.gz` (only README.md differs).
- ROS_DOMAIN_ID=233 is invalid for Fast DDS (max 232; "Calculated port number is too high"). Used 190 (main), 191/192 (parallel test).
- pygame was not installed system-wide on this boot: `pip install pygame==2.6.1` into a throwaway virtualenv, added via PYTHONPATH.
- git HEAD at run time: 130f5c1be14859db9441d1bfa4870deb4e73519e (another agent committed README/GIF during the session; source files unaffected).
- Source fingerprint (same algorithm as run_sighting_experiments.py): default incl. README and my 3 new tools = `62ce15af8be66f4fab8c3de89c4fcf37ad7a30b77f46cf6fd34c9374f916837a`; excluding the 3 new tools = `e39d6c731d754d53a0140b67891a55378d1302d13b97aa553ea2f97739260832`; excluding new tools and README.md = `16af288218d5ba7c18f554e6fb1f23af80fcf029d5c80ee3046086d4b2e42fa8`. (Frozen 2026-10-07 value b7f28c05... differs only because README.md changed.)
- Tools (new): `tools/run_sighting_diagnostic.py` (runner; adds a depth-300 BEST_EFFORT shared-sighting collector, drops the /detections subscriptions of the old collector, records wall/CPU/loadavg), `tools/analyze_sighting_diagnostic.py` (offline analysis; produces summary.json).

Command (main grid, `runs/`):
```
export ROS_DOMAIN_ID=190 SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy
python3 ros2_ws/src/boids_swarm/tools/run_sighting_diagnostic.py --domain 190 --out artifacts/sighting-relay-diagnostic-2026-10-08/runs \
  --cells oracle:4:11,ros:4:11,off:4:11,...(seeds 11,23,37,41,53 x oracle/ros/off at n=4),oracle:12:11,ros:12:11,oracle:12:23,ros:12:23,oracle:12:37,ros:12:37
python3 ros2_ws/src/boids_swarm/tools/analyze_sighting_diagnostic.py runs summary.json
```
Extra folders: `parallel-test/` (first parallel attempt, two n=12 runs crashed), `parallel-test2/` (second parallel attempt), `seq-repeat1/`, `seq-repeat2/` (sequential same-seed repeats). Per run: run_config.json, launch.log, relay_events / local_sightings / shared_sightings_depth1 / shared_sightings_depth300 / observations / track_status .jsonl.

## 1. Outcomes (main grid, `summary.json` -> by_group)
| group | captured/runs | Wilson 95% | capture times | mean wall (s) | mean CPU cores |
|---|---|---|---|---|---|
| oracle n=4 | 0/5 | 0-0.43 | - | 10.1 | 1.4 |
| ros n=4 | 0/5 | 0-0.43 | - | 10.4 | 1.6 |
| off n=4 | 0/5 | 0-0.43 | - | 9.6 | 1.5 |
| oracle n=12 | 2/3 | 0.21-0.94 | 8.62 (s11), 6.05 (s37); s23 timeout | 8.5 | 2.8 |
| ros n=12 | 1/3 | 0.06-0.79 | 8.47 (s11); s23, s37 timeout | 14.4 | 2.6 |

Per-run: outcome, capture time, wall, cpu, track-valid fraction (cycle status with own_pose_fresh; belief_valid) are in summary.json `runs[]`. n=4 track-valid ranges: oracle 0.89-1.00, ros 0.86-1.00, off 0.37-0.91.

## 2. Same-seed repeats (n=12): outcome is NOT deterministic
| cell | attempts (outcome, t) |
|---|---|
| oracle n12 s11 | captured 8.62 / 8.82 / 8.62; parallel-test2: timeout |
| ros n12 s11 | captured 8.47 / 8.60 / 8.93; parallel-test2: captured 8.67 |
| oracle n12 s37 | captured 6.05 / 6.88; timeout |
| ros n12 s37 | timeout / timeout / captured 5.98 |
Seeds fix the world layout and initial state but not the outcome (ROS message timing and wall-clock scheduling feed the dynamics). Captures, when they happen, are early (6-9 s); otherwise the run times out at 30 s.

## 3. Packet loss (ros mode), method and caveats
Every `/swarm/target_sightings` message that reaches a controller callback emits exactly one relay_event (including `self_originated` and `out_of_range`), so the range gate does NOT reduce the expected event count: expected events per published sighting = num_agents (self + others). Confirmed: duplicate (receiver,key) events = 0 in every run.
- published keys (low) = union of keys in the depth-300 BEST_EFFORT collector and in any relay event. Upper bound adds all sim `local_target_sighting` keys (`published_keys_up`, at most ~0.5-4 % higher; direct inbox rejects not separable).
- loss = 1 - relay_events / (num_agents x published_keys_low). Denominator is num_agents x keys (n=4: 4 x 2.3-2.9k per run), not a fixed 30x30x4.
- Confound: relay_events travel over a RELIABLE channel to a single Python collector. Collector loss cannot be separated from receiver loss from this data. Proxy: `status_channel_missing` in summary.json (gaps in control_cycle_sequence of the reliable status topic) is 0-17 % of messages in n=4 ros runs (0 in seed 53). So the observed loss is an UPPER bound of true receiver loss (inference).
- n=4 ros, 5 runs pooled: 29,879 events / (4 x 12,719 keys) -> observed loss 41.3 % (per run 36.2-46.6 %); non-self receivers 48.2 %, self (loopback) 20.3 %. Cleanest run (status_missing = 0, ros-n4-seed53): 38.8 %.
- Direct evidence of depth effect: in the same process, the depth-1 BEST_EFFORT collector saw only 64-71 % of the keys that the depth-300 BEST_EFFORT collector saw (n=4); 47-51 % at n=12. This isolates the history-depth effect (same node, same QoS except depth) and indicates depth=1 BEST_EFFORT is self-inflicted loss (inference: receivers' controllers have the same depth and the same Python executor, plus machine load average 20-28 on 16 cores from other jobs).
- n=12 event-based loss (60-73 %) is dominated by collector overload (18k events/run); do not use it.
- Of events that arrive (n=4): accepted 33-54 % of events per run; remaining mostly `late_observation` (stale > sighting_timeout 0.6 s) and some `out_of_range`.
- Cannot compute from existing logs: "in range and should receive" per receiver (collector records no agent poses). Needed counters (suggestions only, not applied): (a) boid_controller_node.py:274 (`_on_shared_sighting` entry) a per-receiver callback counter published in track status/relay summary; (b) boid_controller_node.py:306 (`_publish_relay_event`) a per-receiver monotonically increasing event index so collector loss is detectable; (c) pygame_sim_node.py:628 (after `sighting_pubs[i].publish`) / boid_controller_node.py:272 (`shared_sighting_pub.publish`) a published counter; (d) log agent poses (or distance matrix) with each sighting for the range denominator.

## 4. Cost and parallelism
- Sim is speed-capped at ~3.5x real time headless (log "speed=3.4-3.8x"). n=4, 30 s episode: wall 8.8-12.0 s (mean ~10), children CPU 12.9-17.5 s (1.2-1.8 cores avg). n=12 timeout: wall 13-20 s, 2.4-3.5 cores; n=12 capture at 6-9 s: wall 4.7-6.6 s.
- Machine was not idle: load average 19-28 on 16 cores from unrelated background load on the machine, so wall times and speed (2.1x in the loaded n=12 run) are pessimistic and noisy.
- Parallel test (domains 191 and 192 simultaneously): attempt 1 (`parallel-test/`): both n=12 sims died at the same moment (~10:57:09) with `RCLError: publisher's context is invalid` (a signal-like context shutdown) -> no result. Cause unknown (inference: external SIGINT or parallel interference; cannot distinguish). Attempt 2 (`parallel-test2/`): all four runs completed cleanly, n=4 wall 9.5-12.5 s (vs ~10 sequential), no tracebacks. Outcomes differed from sequential for oracle n12 s11 (timeout vs captured) but sequential repeats also vary (section 2), so the difference is not attributable to parallelism. Conclusion: 2 in parallel is probably OK for wall time but had one unexplained double crash; treat as not proven safe.

## 5. Other anomalies
- n=4: 0/15 captured across oracle/ros/off at 30 s with ranges aligned to 8 m (frozen run 2026-10-07 with comm_range 12 also 0/9). The 30 s cap and/or 4 agents may make capture unreachable here; n=12 captures within 9 s or not at all.
- Oracle uses multi-hop connected components over `comm_range` (comms.propagate_sightings) while ros mode is one-hop with `radio_range`; aligning both to 8 m does not make the topologies equal.
- The local-sighting collector (reliable, depth 300, single-threaded Python) undercounts at n=12 and sometimes n=4 (e.g. ros-n4-seed11: 2052 local vs 2242 shared keys), so the original collector's "local vs shared matched keys" is not a delivery measure.
- Non-ros modes have no relay events, hence no loss figures; track-valid exists for all modes.
