# Portfolio direction: distributed sighting relay

Date: 2026-10-07  
Target roles: robotics, simulation, and control engineering.

## Three directions

### 1. Move target-sighting relay into real ROS communication — selected

Today, `comms.py` computes connected components from every agent's true position, and `pygame_sim_node._publish_detections()` uses the true target pose to make a receiver-specific relayed detection. A ROS-backed path would let agent A publish its own noisy sighting and agent B consume that same measurement, reject it when stale or outside the configured one-hop radio range, and return to search when no fresh sighting remains. This gives a reviewer an inspectable distributed-control boundary while keeping direct sensing first.

**Red review:** Calling this fully decentralized would overstate the implementation. The simulator still owns ground truth and will supply simulated agent poses used by a receiver-side radio-range gate. The current oracle route also relays a target-truth-derived point. The selected design keeps both facts explicit: `ros` carries A's noisy raw local observation over ROS; the radio-range check uses simulator-published pose data and is an application-layer simulation constraint while DDS can deliver globally, not a radio/network model.

**White review:** Keep this to one hop. Do not add Nav2, consensus, multi-hop forwarding, or a general network simulator before the measurement path and stale-data behavior work end to end.

**Decision:** Adopt one-hop ROS sighting delivery as the core. Keep direct local sensing higher priority than received data. Do not let sensor-mode controllers subscribe to target truth.

### 2. Compare off, oracle, and ROS paths with a small reproducible experiment — selected

Run the same four-agent scenario in three sharing modes: `strategy=intercept`, `env=open`, `game_mode=ai`, `perception=sensor`, `capture_mode=hull`, one 30.0-second episode, and seeds `[11, 23, 37]`. Save the full resolved configuration, source revision, raw direct observations, relay events, and concise statistics. Report simulation-time observation age (not DDS latency), receiver-opportunity coverage, target-track-valid fraction, stale-message rejection, and capture outcome/time as exploratory game metrics.

**Red review:** A capture result from a small seed set cannot establish safety, general superiority, or statistical significance. A single favorable run could be spawn luck. Do not frame capture as a safety guarantee.

**White review:** A compact evidence artifact makes the engineering decision reviewable. Start with only measurements the runtime can actually emit; keep the fixed scenario and seeds visible.

**Decision:** Adopt the small experiment and raw event artifact. Treat capture results as exploratory, not as a safety claim or broad performance proof. Do not run a large sweep in this phase.

### 3. Add delay/dropout and larger network experiments — deferred

Explicit delay, packet loss, changing topology, multi-hop consensus, and parameter sweeps could show robustness under communication faults.

**Red review:** This is a separate network-modeling project. It risks measuring invented simulator behavior before the basic ROS path has passed a real two-node smoke test.

**White review:** We still need one failure-path demonstration. A controlled test can stop A's sighting publisher and check B's stale timeout/search behavior without building a delay/dropout simulator.

**Decision:** Defer network simulation and large sweeps. Cover outage by stopping the publisher in the integration smoke. Revisit network faults after the one-hop path has credible evidence.

## Scope decision

Phase 1 is the ROS-backed one-hop path plus a small, reproducible comparison. It preserves current launch behavior by mapping legacy configuration to the oracle baseline. There is no Nav2, multi-hop consensus, complete network simulation, or large sweep. A polished dashboard is also deferred; a readable README update and machine-readable evidence are sufficient for this phase.

The project profile suggests that a runnable demo, clear technical explanation, and reproducible evidence help an external reviewer judge engineering work. That is a portfolio recommendation inferred from the user's stated job target and profile, not a claim that one implementation choice is already proven to improve hiring outcomes.
