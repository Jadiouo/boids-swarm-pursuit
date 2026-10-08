# Software Design Document — Distributed Sighting Relay v1

Date: 2026-10-07  
Status: implemented for phase 1; verified on CPU in ROS 2 Jazzy. Deferred items remain outside scope: Nav2, multi-hop consensus, network delay/dropout modeling, large sweeps, uncertainty-weighted fusion, and UI reverse synchronization.  
Target: robotics, simulation, and control engineering portfolio evidence.

## 1. Problem and decision

In sensor mode, the simulator currently computes each agent's noisy detections, then `comms.propagate_sightings()` finds transitive connected components from all true agent positions. `pygame_sim_node._publish_detections()` calls `perception.relayed_detection()` with the true target state for receivers that did not see it. The receiving boid controller consumes a per-agent `Float32MultiArray`; therefore the current relay is a simulator oracle, not a ROS exchange between controllers.

Phase 1 adds a real ROS 2 one-hop sighting path. The simulator's per-agent sensor adapter converts A's direct noisy range/bearing measurement into a world-frame point using A's simulated sensor pose, stamps it at observation time, and publishes it only to A. This point is a coordinate-transformed **raw sensor observation**, not a filtered or fused target estimate. A's controller republishes that same observation. Agent B receives A's original measurement, checks its age and application-level radio range, and feeds it into its tracker once. B never republishes a tracker output or a received measurement. Direct local sensing takes priority. A controller never reconstructs a target measurement from `/target/pose` in sensor mode.

The simulator remains the world owner and supplies local sensor measurements and simulated poses. For the `ros` mode, the ROS message carries A's noisy raw observation; the simulator does not calculate a receiver-specific relayed target point. The receiver-side range gate uses simulated agent poses supplied by the simulator. Thus the ROS message path is real, while range enforcement still depends on a simulation pose feed. DDS can deliver the topic globally; `radio_range` is an application-layer simulated communication-range gate. The result is a bounded portfolio demonstrator, not a claim of a fully decentralized physical radio network.

## 2. Existing behavior and constraints

- `boids_swarm` is an `ament_python` ROS 2 package. The workspace has no existing `boids_swarm_msgs` package or `.msg` definitions.
- `pygame_sim_node` publishes `/agentK/pose`, `/agentK/detections`, `/target/pose`, and `/clock`; sensor-mode boid controllers subscribe to their own pose and detections, not `/target/pose`.
- Current direct detections are `Float32MultiArray` with six-float records and no header timestamp. The sensor geometry already injects range/bearing noise.
- `comms.py` computes transitive oracle connectivity. It remains available as a named comparison baseline.
- Existing launch defaults to `perception:=perfect`; `perception:=sensor` currently enables local detections and oracle relay when `comms_enabled` is true.
- Preserve existing `perception:=perfect|sensor` behavior and current launch invocations. Do not change the default behavior in this phase.

## 3. Modes and compatibility

Introduce an explicit `sharing_mode`:

| Mode | Direct local sensing | Sharing behavior | Meaning |
|---|---|---|---|
| `off` | Existing `perception_mode` behavior | No sightings are shared | Sensor-only baseline when `perception_mode=sensor` |
| `oracle` | Existing behavior | Existing simulator relay through transitive true-position connectivity | Explicit legacy comparison baseline; not a claim of decentralization (it uses global truth) |
| `ros` | Required (`perception_mode=sensor`) | A controller publishes its direct noisy raw observation; peers consume one hop over ROS | Phase 1 core |
| `legacy` | Existing behavior | Dynamically follows current `comms_enabled` at each simulator detection publish | Compatibility resolver; never cached as a startup-only choice |

Compatibility rules:

1. Keep `perception_mode:=perfect|sensor` and the existing `comms_enabled` parameter accepted.
2. Add launch `sharing_mode` with a transitional `legacy` value as the default compatibility resolver. In `legacy`, the simulator checks the current `comms_enabled` parameter whenever it publishes detections: with sensor perception, `true` selects oracle relay and `false` selects off. Do not resolve this once at launch because the parameter is runtime-changeable today.
3. Explicit `sharing_mode:=off|oracle|ros` takes precedence over and ignores runtime changes to `comms_enabled`; if both are explicitly set inconsistently, log one clear deprecation/configuration warning. Do not silently reinterpret explicit `sharing_mode`.
4. Reject `sharing_mode=ros` with `perception_mode=perfect`, since that mode has no direct noisy local sighting to transmit.
5. Keep simulator `/target/pose` for game physics and the existing target controller. Sensor-mode boid controllers must not subscribe to it.

## 4. Message and topic contract

No reusable `boids_swarm_msgs` interface exists. A new `boids_swarm_msgs` ROS interface package is justified because the contract needs explicit sender, episode, measurement age, measurement uncertainty, and source pose fields; encoding these in an untyped float array or overloading `frame_id` would make the critical semantics hard to review.

Add `boids_swarm_msgs/msg/TargetSighting.msg`:

```text
std_msgs/Header header
string sender_id
uint32 episode_id
uint64 sequence
string target_id
geometry_msgs/Point sender_position
geometry_msgs/Point target_position
float64[4] covariance_xy
float32 confidence
float32 valid_for_sec
```

Also add `boids_swarm_msgs/msg/EpisodeState.msg`:

```text
std_msgs/Header header
uint32 episode_id
```

Contract:

- `header.stamp` is the direct target observation time in ROS/simulation clock; `header.frame_id` is exactly `world`.
- `sender_id` is the originating agent namespace (for example `agent0`); `target_id` is `target` in this single-target phase.
- `sequence` is a per-sender, strictly increasing `uint64` assigned to each direct target observation within an episode. It resets only when the authoritative episode changes. Receivers use `(episode_id, sender_id, sequence)` for de-duplication and reject a reused sequence with a different payload as a conflict.
- For a given `(episode_id, sender_id)`, a new direct observation must also have a strictly later `header.stamp`. Same sender and same stamp with a different sequence or payload is a `sender_stamp_conflict` and is rejected; observations from different senders may legitimately share a stamp.
- `sender_position` is A's simulated local odometry position **at the observation timestamp**, from the same simulation sample used to synthesize the noisy measurement. Do not replace it with A's later publication-time position. The simulator's pose feed is globally visible to ROS nodes; its use for range enforcement must be disclosed as a simulation aid.
- `target_position` is the coordinate transform of A's **noisy raw** range/bearing observation into `world` using A's simulated sensor pose. The sensor adapter performs only this frame conversion, not target-truth lookup, filtering, or fusion. It must not be populated from the true target state or recomputed separately for B.
- `covariance_xy` is the row-major 2×2 measurement covariance `[xx, xy, yx, yy]` in square world units, derived from the sensor noise model. It describes this raw observation, not a fused track. It must be finite, symmetric within tolerance, and positive semidefinite.
- `confidence` is finite in `[0,1]`; it is descriptive, not a safety probability.
- `valid_for_sec` is positive and bounded by a configured maximum. A receiver uses the minimum of this value and its configured `sighting_timeout`.
- `episode_id` is assigned only by the simulator through `/simulation/episode_state`; increment on every game reset and clock-epoch reset. Never adopt or advance an episode from a `TargetSighting` packet.
- `header.stamp` and `sender_position` refer to the same observation-time sample. B compares the sender sample with its own cached `/agentB/pose` sample whose callback stamp is in ROS simulation time. Accept the range decision only if that local pose sample is not older than `pose_timeout` and differs from the sighting timestamp by at most `radio_pose_time_tolerance`; otherwise reject with `receiver_pose_unavailable` or `receiver_pose_time_mismatch`. The range calculation is Euclidean distance in `world`, with no interpolation or history buffer in this phase.
- `EpisodeState.header.stamp` is the simulator-clock reset stamp and `frame_id=world`. The simulator is the sole publisher on `/simulation/episode_state`; it publishes at startup and every reset using `KEEP_LAST(1)`, `RELIABLE`, `TRANSIENT_LOCAL`. Controllers subscribe with matching reliable/transient-local QoS so late joiners receive the current episode even before any target sighting exists.
- On each authoritative episode change, controllers immediately clear all cached poses, direct/shared candidate queues, target and neighbor tracks, target freshness, sequence/dedup state, and latest target status; they wait for fresh pose and sensing data. Publish an immediate public status event with `belief_valid=false`, `mode=search`, and `reason=episode_reset` before the next control cycle, even when no target sighting follows. Until the first authoritative episode state arrives, sightings are rejected. A sighting with old or future `episode_id` is rejected and cannot change the adopted epoch.
- If a controller detects ROS clock rollback within an episode, it immediately clears those same buffers and tracks, marks itself `awaiting_episode_state`, and rejects all sightings until a fresh authoritative episode-state event arrives. The simulator must publish the new episode state before publishing detections after a reset. A packet whose timestamp is in the future beyond configured tolerance is rejected.

Topics:

| Topic | Type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `/agentK/local_target_sighting` | `TargetSighting` | simulator | that agent's controller | Stamped direct noisy observation; only present for a local detection |
| `/swarm/target_sightings` | `TargetSighting` | each controller | all controllers | One-hop ROS delivery; receivers do not republish received data |
| `/simulation/episode_state` | `EpisodeState` | simulator only | all controllers | Authoritative current epoch/reset; reliable transient-local for late joiners |
| `/agentK/pose` | existing simulated pose | simulator | controllers | Own pose and explicit simulation pose source for range gating |
| `/swarm/observations` | `std_msgs/String` JSON | simulator | experiment capture | Direct noisy observation telemetry; not a control input |
| `/swarm/relay_events` | `std_msgs/String` JSON | each controller | experiment capture | Per-receiver shared-sighting decision and reason |
| `/agentK/target_track_status` | `std_msgs/String` JSON | each controller | experiment capture/tests | Per-cycle track/search status plus reset/clock transition events |

In `ros` mode, the legacy `/agentK/detections` array is filtered to contain neighbors only; remove target records so the same sensor frame cannot enter the tracker through both the legacy array and `/agentK/local_target_sighting`. The stamped message is the sole direct target-observation path in that mode. `off` and `oracle` retain their existing target-array semantics for compatibility.

## 5. Freshness, validation, and radio-range behavior

Use `KEEP_LAST(1)`, `BEST_EFFORT`, and `VOLATILE` QoS for `/swarm/target_sightings`. These are latest-state observations; stale backlog is less useful than a missed old packet. Publisher and subscriber profiles must match. The subscriber still validates every message because QoS is not data validation.

Reject and count a message when any of the following applies:

- malformed sender/target/frame IDs, non-finite coordinates/covariance/confidence/validity, invalid covariance, or non-positive validity;
- self-originated message, duplicate `(episode_id, sender_id, sequence)`, same sequence with a different payload, same sender/stamp with a different sequence or payload, or non-increasing sequence/stamp for that sender;
- `episode_id` differs from the latest authoritative `EpisodeState`, timestamp is in the future beyond tolerance, or age exceeds `min(valid_for_sec, sighting_timeout)`;
- sender observation-time pose or receiver own pose is unavailable/stale, their sample-time difference exceeds `radio_pose_time_tolerance`, or their Euclidean distance exceeds `radio_range`.

All sensor callbacks only enqueue candidate observations/detections; they never call `Tracker.step()` or `_note_target()`. At each control tick, atomically drain the candidates that arrived before the tick cutoff, collapse exact `(episode_id, sender_id, sequence)` duplicates, and reject same-sequence payload conflicts. If conflicting payloads with the same key are present in one drained batch, discard that key rather than selecting by arrival order. Keep at most the highest sequence per sender from the drained batch. Choose exactly one target observation for that cycle: a valid local direct observation always beats any received observation in that drained batch; otherwise choose the valid received observation with the newest `header.stamp`, breaking ties by `sender_id` for deterministic behavior. Direct priority applies to candidates available at the tick cutoff; a late callback never retroactively rewrites an applied tick. Reject any candidate whose stamp is at or before the last applied target-observation stamp, so a late callback cannot cause a second update for an already processed sensor frame. Combine the chosen single target observation with the latest neighbor detections and call the tracker at most once per control tick. Never average multiple candidates in this phase.

Keep the original observation timestamp and measurement covariance through receipt. Pass the selected `header.stamp` as target observation time into tracking; keep control time separate for prediction/expiry. Never stamp `_note_target()` with callback receipt `now`. Freshness is calculated from `header.stamp`, not callback receipt time, control tick time, or a replacement stamp. The selected target observation is applied at most once; any later callback with a stamp at or before the last applied observation is rejected. A received sighting does not extend its lifetime, become a new sender, or get forwarded again. If direct sensing and a fresh relay coexist in the drained batch, direct local observation wins. If no direct or received observation remains fresh, clear target freshness and use the existing sensor-mode search fallback. Do not rely on `Tracker.max_age` to implement sighting TTL: when `now - last_observation_stamp > min(valid_for_sec, sighting_timeout)`, explicitly clear the target belief and enter search even if callbacks for neighbor detections continue.

`covariance_xy` and `confidence` are validated and preserved for telemetry/diagnostics only in phase 1. The current tracker API does not weight updates by either value; this design does not claim uncertainty-aware fusion. Use one selected measurement per control tick to avoid pretending that the tracker performs covariance fusion.

The radio-range rule is application-level filtering: DDS can deliver the topic to every ROS subscriber. It is not a model of physical RF reachability, packet delay, or packet loss. The range is checked using sender and receiver pose samples from the simulator. Logs and README language must preserve this limitation.

## 6. Runtime and experimental evidence

Add configurable `sharing_mode`, `radio_range`, `radio_pose_time_tolerance`, `sighting_timeout`, `future_stamp_tolerance`, and a validation/outcome counter interface. Keep existing direct sensor configuration. In `legacy` mode, check current `comms_enabled` whenever the simulator publishes detections: `true` means the current oracle path is active and `false` means sharing is off. Do not resolve this once at launch. Explicit `off`, `oracle`, or `ros` modes ignore later `comms_enabled` changes. Freeze experiment fields in a runner/config: strategy, environment, capture mode, episode timeout, seed list, perception settings, sharing mode, radio range, and source provenance.

For reviewable telemetry, use `std_msgs/String` JSON records on three public topics rather than adding a diagnostics framework:

- Simulator publishes `/swarm/observations` once per direct target detection, with `episode_id`, `sender_id`, `sequence`, observation stamp, raw noisy range/bearing, transformed `target_position`, measurement covariance, and confidence. In the real ROS smoke, match this record to the wire `TargetSighting` by `(episode_id, sender_id, sequence)` and verify target payload, stamp, sender pose, and covariance are unchanged. A test harness may separately observe target truth to assert the noisy point differs; truth must never be included in `TargetSighting` or ordinary portfolio artifacts.
- Controllers publish `/swarm/relay_events` for each shared sighting decision, including receiver/source IDs, episode/sequence, observation and receive stamps, age, range distance, and a stable accept/reject reason.
- Each controller publishes `/agentK/target_track_status` once per control cycle with `episode_id`, control-cycle stamp/sequence, `belief_valid`, mode (`direct`, `relay`, or `search` in ROS mode; `oracle_tracking` or `legacy_tracking` may appear for those baselines), last applied source/sequence/observation stamp when available, and reason. Baseline modes set `source=unknown`, because this interface cannot distinguish local from oracle-derived tracking. It also publishes an immediate `event=transition` on authoritative reset or clock rollback; reset reports `belief_valid=false`, `mode=search`, and `reason=episode_reset` before any new target observation. This is the public seam for proving search and reset behavior; do not inspect private attributes in tests.

The runner subscribes to these topics and the simulator's episode summary, and writes JSONL/JSON artifacts. Compute metrics per episode, then aggregate:

- delivery delay = receive stamp − original observation stamp for accepted relay events in the same ROS clock epoch;
- recipient coverage = distinct receivers with at least one accepted relay divided by distinct receivers with at least one in-range, same-episode, fresh relay opportunity, counting each receiver once per episode; derive opportunities from public relay-decision events and report `null` when the denominator is zero;
- target-track-valid fraction = `belief_valid=true` status records divided by all post-episode-start status records with a fresh own pose, including search cycles; the explicit denominator counts one status per control cycle;
- stale rejection count = count of relay events with stale/old/future timestamp reasons; separate other reject reasons;
- capture outcome/time = simulator-reported game result, exploratory only.

Episode reset clears per-episode metric counts. Run provenance includes git commit, `working_tree_dirty`, and a deterministic `source_fingerprint_sha256` over the relevant source/config/test files including untracked files (excluding `.git`, venvs, build/install/log directories and generated result artifacts), plus ROS/Python/package versions and resolved parameters. Commit alone is not sufficient when the tree is dirty; fingerprint is a verification identifier, not a substitute for preserving the actual source tree.

Phase 1's compact comparison runs `off`, `oracle`, and `ros` with a fixed scenario: four agents, `strategy=intercept`, `env=open`, `game_mode=ai`, `perception=sensor`, `capture_mode=hull`, one episode with a 30.0-second timeout, and seeds `[11, 23, 37]`. Keep all remaining resolved parameters identical and record them. Save:

- `run_config.json`: full resolved parameters, seed, ROS/package versions, git commit, dirty flag, source fingerprint, and run status;
- `observations.jsonl`: raw stamped direct noisy target observations emitted by the simulator;
- `relay_events.jsonl`: source/receiver IDs, episode/sequence, original measurement stamp, receive stamp, age, radio distance, and accept/reject reason;
- `track_status.jsonl`: public per-control-cycle status events used to calculate track-valid fraction and confirm search/reset;
- `summary.json`: per-episode and aggregate metric values with explicit denominators, actual run count, and capture outcome/time where available.

Do not synthesize results or describe capture as a safety metric. Do not promise bitwise determinism: seeds and full configuration make a run reproducible enough to inspect, while ROS scheduling and real wall-clock transport may vary.

## 7. Planned file/module changes

Implementation scope for the later coding phase:

- New `ros2_ws/src/boids_swarm_msgs/`: a small `ament_cmake`/`rosidl` package containing `TargetSighting.msg`, `EpisodeState.msg`, and package metadata.
- `ros2_ws/src/boids_swarm/boids_swarm/perception.py`: expose the direct noisy target measurement and covariance transformation without using the target's true pose for relay payloads.
- `ros2_ws/src/boids_swarm/boids_swarm/pygame_sim_node.py`: publish stamped local target observations and authoritative episode state; keep runtime `legacy` resolution and explicit oracle baseline; filter target out of the legacy detection array in `ros`; skip oracle relay generation in `ros` and `off` modes.
- `ros2_ws/src/boids_swarm/boids_swarm/boid_controller_node.py`: callbacks enqueue only; at most one target observation is selected/applied per control tick, with direct priority, sequence de-duplication, and no relay-derived forwarding; publish public relay/track telemetry; clear all state on authoritative reset and clock rollback.
- `ros2_ws/src/boids_swarm/boids_swarm/tracking.py` or a focused new pure helper module: accept/update validated observations using the original observation timestamp, and clear state on TTL/reset, without ROS dependencies where practical. Covariance/confidence remain diagnostic only.
- `ros2_ws/src/boids_swarm/launch/pursuit.launch.py` and `config/params.yaml`: modes/parameters and backwards-compatible resolution.
- `ros2_ws/src/boids_swarm/package.xml`, `setup.py`, workspace package metadata as required by the message dependency.
- `ros2_ws/src/boids_swarm/test/`: focused pure validation tests and ROS integration tests.
- `ros2_ws/src/boids_swarm/README.md` and root `README.md`: correct description of oracle versus ROS path, launch usage, known localization/radio limitations, and real run evidence only.
- `ros2_ws/src/boids_swarm/tools/` (or similarly named module): small fixed-config experiment runner and output schema.

Do not refactor the existing control panel in this phase. Its reverse synchronization behavior is a separate known debt.

## 8. Non-goals

- Nav2, path planning, or changes to pursuit tactics.
- Multi-hop relay, consensus, gossip, or forwarding received messages.
- A full network simulator, explicit delay/dropout model, or large parameter/seed sweep.
- Claims of decentralized localization or physical radio modeling.
- Capture as a safety guarantee; bitwise deterministic execution.
- Control-panel reverse synchronization or unrelated UI changes.
- GPU computation. All planned validation and smoke runs are CPU-only and ROS headless.

## 9. Acceptance criteria

- **SDD-01:** Existing launch invocations retain their current `perfect`/`sensor` behavior; `legacy` dynamically follows `comms_enabled`; explicit `off`, `oracle`, and `ros` modes are stable and ignore it.
- **SDD-02:** The `ros` mode transmits A's direct noisy raw observation through ROS without simulator-generated target-truth relay payloads or tracker-output forwarding; controllers in sensor mode do not subscribe to `/target/pose`.
- **SDD-03:** A's noisy observation is transmitted unchanged in target content, source, sequence, stamp, observation-time sender pose, and measurement covariance; the ROS path is the sole target input in `ros` mode and each control cycle applies at most one target observation. Same-tick candidates use direct-over-relay priority; later callbacks cannot cause a second update.
- **SDD-04:** Invalid, duplicate/conflicting-sequence, out-of-order, stale, future, old/future-episode, clock-rollback, late-callback, and out-of-range cases are rejected and counted; TTL is based on observation time, and stale loss returns B to search even while neighbor detections continue.
- **SDD-05:** A CPU-only headless ROS integration smoke with two controller nodes demonstrates A local observation → ROS message → B track/control response, publisher outage → TTL expiry/search while neighbor detections continue, same-frame direct-priority/dedup, reset with no sightings, old/future-epoch rejection, clock-rollback clearing, and late-join episode-state receipt.
- **SDD-06:** A small fixed-config comparison produces actual config, raw observations matched to wire payloads, relay events, per-cycle track status, explicit metric denominators, dirty-state/source-fingerprint provenance, and summary artifacts for all configured modes/seeds; README claims match the artifacts and limitations.

## 10. 已知限制(2026-10-08 審查)

- `ros` 模式是 ROS 2 一跳 topic 傳輸加應用層範圍閘;range gate 使用模擬器提供的 pose 與 sender 自報位置,不是完全分散式,也不是無線電模型。
- 實驗 `artifacts/sighting-relay-final-2026-10-07`:`oracle` 用 `comm_range` 12 m 多跳(`comms.py:53` DSU 連通分量),`ros` 用 `radio_range` 8 m 單跳,兩者不可直接比較。
- 該實驗 12 個 run 全部 30 秒 timeout、零捕獲;只有 3 個 seed,同 seed 不可重現。
- shared sighting QoS 為 BEST_EFFORT、depth=1。（註：phase 1 當時的預設；E1 之後出貨預設改為 depth=10，depth=1 仍可用參數指定。）
- Phase 2 將以對齊 range、預先登記場景重做評估。
