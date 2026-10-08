# artifacts index

One row per folder. Status: **conclusion** (pre-registered or final evidence you may cite), **diagnostic** (post-hoc or supporting, not a hypothesis test), **exploration** (feasibility/tuning, not a claim), **voided** (do not cite).

Raw per-run data (`runs/`, logs, JSONL) is git-ignored; only each folder's `README.md`, `SUPERSEDED.md` and `summary.json` are tracked (see `.gitignore`).

**Historical paths.** Absolute machine-specific paths (temporary directories, home directories) were removed from the tracked files before publication; the `config` field of the `nav2-m7-tuning/after-red-team/` JSON files now holds only the config file name. Raw `run_config.json` files and logs, which are git-ignored, may still contain original paths. Reproduction commands in the READMEs use repo-relative paths (see `scripts/quickstart.sh`).

| folder | purpose | status | date | key number | docs |
|---|---|---|---|---|---|
| `_superseded-sighting-relay-early-2026-10-07/` | First, abandoned round of the one-hop sighting experiment (seeds not pinned, schema differs) | voided | 2026-10-07 | none; numbers contradict the final round | `SUPERSEDED.md` in folder |
| `sighting-relay-final-2026-10-07/` | Frozen evidence for off / oracle / ROS one-hop sighting sharing (3 seeds + 3 repeats, 4 agents, 30 s) | conclusion (pipeline works; no performance ranking) | 2026-10-07 | 12/12 runs completed, 0 captures, so no capture-rate claim | `SDD/Sdd_distributed_sighting_relay_v1.md`, folder `README.md` |
| `sighting-relay-diagnostic-2026-10-08/` | CPU-only diagnostic used to design phase 2: delivery/drop accounting, parallel vs sequential runs | diagnostic | 2026-10-08 | n=4: 0/5 captures in every mode; n=12: oracle 2/3, ros 1/3 | `SDD/Sdd_relay_phase2_prereg.md`, folder `README.md` |
| `relay-e1-2026-10-08/` | Pre-registered E1: shared-sighting QoS depth 1 vs 10, vs oracle and off (150 runs, 0 invalid) | conclusion | 2026-10-08 | S12 in-range drop 48.8% (depth 1) vs 1.8% (depth 10); capture-rate ordering not detectable | `SDD/Sdd_relay_phase2_prereg.md`, folder `README.md` |
| `relay-e1b-followup-2026-10-08/` | E1b post-hoc follow-up: does depth 10 turn drops into delay; fast-forward vs real time | diagnostic (not pre-registered) | 2026-10-08 | fast-forward d10 drop 7.7% vs real time 0.3%; 8 or 4 runs per cell, rough | folder `README.md` |
| `nav2-spike/` | Feasibility spike for a Nav2 evader (planner + MPPI controller on an occupancy grid) | exploration | 2026-10-08 | summary.json only: plan success and latency (about 14 ms mean) per scenario | `docs/testing/nav2-mppi-tuning.md` |
| `nav2-m7-tuning/` | Controller tuning sweep (MPPI variants vs RPP, incl. failed combinations) | exploration (revised after red-team review) | 2026-10-08 | shipped controller is RegulatedPurePursuit, not MPPI; per-variant results in `summary.json` and `after-red-team/` | `docs/testing/nav2-mppi-tuning.md` |
| `nav2-m7-sanity/` | Sanity benchmark of `evader:=nav2` vs reactive, seeds 1-6, two repeats | exploration (explicitly not an experiment) | 2026-10-08 | the two repeats of the same code disagree more than the evaders differ; no claim nav2 is stronger or weaker | folder `README.md`, `docs/testing/nav2-mppi-tuning.md` |
