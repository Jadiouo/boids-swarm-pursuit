# TDD plan — Distributed sighting relay

Date: 2026-10-07  
Status: phase 1 implementation and verification complete; final evidence recorded below.

## Confirmed test seam

The approved seam is: **「追捕者A的帶噪觀測經真ROS送到B，B保留量測、驗證新鮮度，過期失聯後回到搜尋」**. The integration path must use actual ROS publishers/subscribers and two controller/relay nodes; a mocked publisher alone does not satisfy it. The already-authorized seam is confirmed for this phase, so no further approval gate is needed.

The TDD skill's exact rule is: **“Test only at pre-agreed seams. Before writing any test, write down the seams under test and confirm them with the user. No test is written at an unconfirmed seam.”** This plan records the seam above and its prior authorization.

Other applicable TDD rules: test behavior through public interfaces; use vertical slices (one failing test, one minimal implementation, repeat); red before green; avoid tests coupled to private implementation, tautological expectations, and bulk tests written before the behavior is understood. Consult `CONTEXT.md` if present and respect relevant ADRs. Refactoring belongs in review, not in the red→green cycle.

## Test environment and baseline

- ROS 2 Jazzy is available after `source /opt/ros/jazzy/setup.bash`.
- Build the workspace message interface before importing the ROS integration package: `cd ros2_ws && colcon build --symlink-install --packages-up-to boids_swarm --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.
- Test collection must set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` to avoid the machine-wide `pytest_metadata` plugin failure.
- Existing baseline: `cd ros2_ws && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` — 135 tests passed before this change.
- pygame (2.6.1 when this was written) must be importable by `/usr/bin/python3`, the shebang of the installed ROS console scripts: `sudo apt install python3-pygame`, or `pip install --user --break-system-packages pygame`. Keep `ROS_DOMAIN_ID=223`, `SDL_VIDEODRIVER=dummy`, and `SDL_AUDIODRIVER=dummy` for isolated headless smoke runs.
- No GPU is needed. Do not run the smoke through a GPU tool or add a GPU dependency.

## Vertical red→green slices

| Slice | Acceptance IDs | Failing behavior test first | Minimal implementation to turn green |
|---|---|---|---|
| 1. Message contract and pure validation | SDD-02, SDD-04 | Reject invalid frame/values/covariance/identity, zero or regressing sequence/stamp, old/future epoch, same-sequence conflicting payload, and same-sender same-stamp/different-sequence payload. | Add `TargetSighting`/`EpisodeState` interfaces and pure validators; do not add network simulation. |
| 2. Authoritative episode lifecycle | SDD-03–05 | Through public ROS topics, assert startup/reset `EpisodeState` is reliable/transient-local; late join receives current epoch; after a valid B belief, publish new EpisodeState and no target sightings; assert an immediate target-track status event on the receiver namespace (the integration fixture uses `/agent1/target_track_status`; `/agentK/target_track_status` in the SDD denotes this per-agent pattern) reports `belief_valid=false`, `mode=search`, `reason=episode_reset` before the next control tick. Also assert old/future sightings cannot promote epoch and clock rollback clears state and waits for authority. | Add simulator-owned episode state and receiver reset/await-authority behavior. |
| 3. Raw measurement and no oracle path | SDD-02, SDD-03, SDD-06 | In actual sim ROS path, match `/swarm/observations` and wire sighting by `(episode_id,sender_id,sequence)`; assert point/stamp/observation-time sender pose/covariance are unchanged, noisy point differs from test-only truth, and ROS-mode legacy detection array contains no target. An injected measurement test alone does not prove oracle relay was cut. | Publish stamped noisy observations only to the observer; remove target from legacy arrays in `ros`; relay original observation unchanged. |
| 4. Batch arbitration and one-hop gate | SDD-01, SDD-03–04 | At public control-cycle seam, same-tick direct beats relay; the old array has no target in actual `ros` mode; duplicate delivery causes one update; conflicting same sequence or same-sender same-stamp payload is rejected; a late callback cannot update the same/older frame twice; out-of-range/stale/self is rejected; B never republishes received data. | Callbacks enqueue only; control tick deduplicates/conflict-checks, picks direct or newest relay deterministically, and applies at most one target observation. |
| 5. TTL and loss fallback | SDD-03–05 | B never sees target, receives one A observation, then A stops sending while neighbor detections continue. At observation-time TTL expiry public status enters/stays search; callback receipt and tracker max-age cannot renew belief. | Preserve source observation stamp through tracker updates; explicitly clear on TTL and expose public track/search status. |
| 6. Explicit modes and runtime legacy compatibility | SDD-01 | Toggle `comms_enabled` while in `legacy`, verify oracle/off switches at runtime; toggle it under each explicit mode and verify no change; reject `ros` with perfect perception. | Resolve `comms_enabled` at every sim detection publish only in legacy mode. |
| 7. Telemetry, metrics, and provenance | SDD-06 | Subscribe to JSON topics; verify observation/relay/status events, per-episode metric numerator/denominator/reset, dirty flag and source fingerprint include modified and untracked source inputs. | Add minimal public JSON telemetry and compute metrics from those records, never private state. |
| 8. Evidence and documentation truth | SDD-06 | Validate output schema with synthetic fixture, then run configured experiment once and inspect artifacts; do not invent results. | Save actual artifacts and update README only with observed evidence and limitations. |

## Validation commands and expected outcomes

Run commands only after implementation and interface build. The pytest plugin setting applies to each pytest invocation:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=223 SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy
cd ros2_ws
colcon build --symlink-install --packages-up-to boids_swarm --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/boids_swarm/ros_test/test_sighting_ros_integration.py -q
```

Expected: message package and `boids_swarm` build; focused tests cover all validation/sequence outcomes; two-node integration tests pass authoritative reset/late join, publisher outage, direct priority, dedup/conflict, old/future epoch, rollback, and no-double-update cases; full suite passes with no plugin-autoload failure.

The CPU-only manual smoke should launch the pursuit launch with `headless:=true`, `ui:=false`, `perception:=sensor`, and `sharing_mode:=ros`, two agents, one bounded episode, and a fixed seed. The environment setup verified this baseline launch command (before the new mode exists):

```bash
timeout --signal=INT --kill-after=5s 45s ros2 launch boids_swarm pursuit.launch.py \
  headless:=true ui:=false num_agents:=4 episodes_max:=1 time_limit:=2.0 \
  seed:=11 strategy:=auto perception:=sensor env:=open
```

After implementing the ROS mode, add `sharing_mode:=ros` and configure the integration fixture to use two agents. Match A's raw observation with its unchanged wire payload, observe B's track and public status, then stop A's publisher while keeping neighbor detections active and verify search from the original observation stamp. After B has a valid belief, reset to a new episode with no target sightings and assert immediate public reset status; also verify a late join receives the current epoch and an old-epoch sighting cannot restore belief. A separate short oracle run confirms the baseline. **Do not rely on process exit code alone:** the current sim can exit 0 when pygame is missing. Inspect `SUMMARY`, observation/relay/status events, and process survival through the bounded episode. Do not report completion until the smoke and one configured experiment actually ran and artifacts were inspected.

The small experiment is not a unit test: run four agents with `strategy=intercept`, `env=open`, `game_mode=ai`, `perception=sensor`, `capture_mode=hull`, one 30.0-second episode, and seeds `[11, 23, 37]` for each of `off`, `oracle`, and `ros`. Record all resolved parameters and emit actual raw observations, relay events, version data, and summary statistics. Capture outcome is exploratory only. No large sweep or bitwise-determinism claim.

## Scope boundary

This plan covers the direct noisy observation → actual ROS one-hop delivery → receiver freshness/range gate → stale fallback path. It excludes Nav2, multi-hop consensus, complete network delay/dropout simulation, large sweeps, UI reverse synchronization, and GPU work. The sim-published pose feed used for the range gate must remain visible in the docs and demo narrative; passing the smoke does not establish physical RF performance or full decentralized operation.


## Implementation record

- Slice 1 red→green: added `TargetSighting`/`EpisodeState` and pure validation. A covariance counterexample with determinant `-1e-10` exposed an absolute-tolerance PSD bug; the validator was corrected to a scale-aware minimum-eigenvalue check. Protocol tests: 13 passed.
- Slice 2–6 red→green: added authoritative episode generation/reset, inbox arbitration, direct-over-relay priority, deduplication, timestamp freshness, app-layer range gate, runtime `legacy` resolution, and independent target tracking. Pure workspace suite includes all new tests and passed: `cd ros2_ws && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q` — 156 passed.
- Real ROS behavior test: `cd ros2_ws && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/boids_swarm/ros_test/test_sighting_ros_integration.py -q -s` — 2 passed (latest local run 3.45s). It uses separate controller processes and public status/cmd_vel topics; verifies one-hop receipt, direct observation use while relay is also published, TTL/search while neighbor detections continue, reset with no sightings, epoch rejection, and late-join reset delivery. Fixture drains callbacks until predicate/deadline to prevent test-side backlog. The test does not control DDS callback arrival order; same-tick arbitration/direct priority is independently verified through the public pure inbox state seam, while ROS verifies direct and relay paths and subsequent control behavior.
- Build: `source /opt/ros/jazzy/setup.bash && cd ros2_ws && colcon build --symlink-install --packages-up-to boids_swarm --base-paths src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` passed. Explicit Python selection is needed on this host because Jazzy uses Python 3.12 while the default user Python is 3.11.
- The simulator experiment runner stores `observations.jsonl`, canonical `local_sightings.jsonl` and `shared_sightings.jsonl`, `relay_events.jsonl`, `track_status.jsonl`, resolved run configuration, launch log, and a source snapshot matching the hashed source inputs. Observation age is simulation-time age, not DDS wall-clock latency. Receiver-opportunity coverage is a receiver-level coverage statistic, not packet delivery ratio.
- Final bounded experiment: 12/12 configured runs completed (9 matrix runs plus one seed-11 repeat per mode), all with timeout at 30.02 simulation seconds; no capture occurred. This is an observed outcome, not a strategy ranking. Per-mode base-seed track-valid fractions were off 0.352/0.780/0.676, oracle 0.928/0.974/1.000, and ROS 0.837/0.938/0.908. ROS base accepted 2,574/2,866/2,943 relay observations; receiver-opportunity coverage was 4/4 in each. Local-to-shared canonical payload matches were 1,015/1,139/1,102; ROS legacy target-array entries were zero in all three runs. Seed-11 repeat track-valid fractions were off 0.360, oracle 0.928, ROS 0.842.
- Artifact root: `artifacts/sighting-relay-final-2026-10-07/` (about 40.7 MB); it includes all per-run logs/configs, observations, local/shared sighting JSONL, relay events, statuses, root aggregate, and the 41-file `source_snapshot.tar.gz` (83,912 bytes). All run fingerprints match the current source fingerprint `b7f28c058e87b00ce14d0c6b192832c458e02c5536598a5abc517f6c26204acc`.
- ROS simulation-time observation-age means for base seeds 11/23/37 were 45.3 μs / 221.0 μs / 96.3 μs; max ages were 16.7 ms / 116.7 ms / 16.7 ms. These are simulation-clock differences, not wall-clock DDS latency; they are quantized near 1/60-second simulation steps. The runner's `zero_fraction` is exact floating-point equality, so tiny residuals (~3.6e-15 sec) are not counted as zero; interpret it as exact-zero only, not same-tick percentage. Coverage is distinct receiver opportunity coverage, not packet delivery ratio. Capture is exploratory only.
