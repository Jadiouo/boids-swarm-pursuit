# Evader comparison: reactive vs adaptive vs smart (2026-10-08)

**Not pre-registered. Behaviour sanity check only. n is small (10 + 5 seeds per
cell). Do not cite as evidence that one evader is stronger than another.**
The evader weights of `smart` were tuned on these arenas (see
`docs/testing/smart-evader-tuning.md`), so `main/` is not independent of the
tuning; `holdout/` (seeds 11-15) was run once, after the final config was fixed.

## Setup

- `tools/evader_compare.py` (one headless CPU run per cell, no GPU, no Nav2).
- 12 agents, `perception:=perfect`, `sharing_mode:=legacy`, strategy `auto`,
  default sim parameters (`target_omega_max` 1.2, speed 1.8x, stamina on,
  `time_scale` 4), 30 s limit, `env` in {obstacle_field, open}.
- `evader` in {reactive, adaptive, smart}; seeds 1-10 (`main/`), 11-15
  (`holdout/`). `obstacle_field` is a fixed layout, so the seed only changes
  spawn positions and sim randomness.
- Each run is **episode 2 of 2** (`episodes_max:=2`). Episode 1 is a warm-up:
  the sim starts its clock while the controller processes are still
  importing, so a single-episode run mostly measures start-up timing
  (target frozen for seconds at 4x). A first sweep without the warm-up was
  discarded for that reason.
- touch = pose within body radius + 0.03 of a wall / obstacle surface (the
  sim clamps exactly there); near = pose within 0.73 of the surface (the band
  the earlier diagnostic recorder used). Wall and obstacle fractions are
  added (a pose can be both). stuck = >= 2 s at speed < 0.5.
- Many episodes end in a few seconds (a capture needs 3 pursuers around the
  target), so per-run fractions are noisy; both the per-run median (range) and
  the time-weighted pooled fraction are given.
- The dev machine was heavily loaded by other jobs during these runs (load
  average 50-85 on 16 cores). Controllers run on sim time, so this adds
  noise to all evaders.

## Results (touch / near are wall + obstacle, % of samples)

### main, seeds 1-10

| env / evader | touch med (range) | near med (range) | pooled touch / near | mean speed | captured / 10 | capture t median | stuck |
|---|---|---|---|---|---|---|---|
| obstacle_field / reactive | 33% (0-62) | 104% (75-140) | 33% / 106% | 2.63 | 10 | 8.4 s | 0 |
| obstacle_field / adaptive | 84% (0-146) | 97% (31-156) | 88% / 105% | 2.79 | 6 | 10.4 s | 0 |
| obstacle_field / smart | 14% (0-52) | 55% (17-103) | 17% / 56% | 2.89 | 9 | 3.3 s | 0 |
| open / reactive | 13% (0-21) | 62% (28-82) | 10% / 72% | 2.60 | 5 | 5.6 s | 0 |
| open / adaptive | 89% (0-93) | 90% (0-94) | 88% / 90% | 2.89 | 1 | 4.4 s | 0 |
| open / smart | 0% (0-0) | 0% (0-16) | 0% / 1% | 3.12 | 7 | 6.2 s | 0 |

### holdout, seeds 11-15 (final config, run once)

| env / evader | touch med (range) | near med (range) | pooled touch / near | mean speed | captured / 5 | capture t median | stuck |
|---|---|---|---|---|---|---|---|
| obstacle_field / reactive | 43% (13-91) | 102% (93-137) | 55% / 115% | 2.77 | 5 | 12.0 s | 0 |
| obstacle_field / adaptive | 85% (0-152) | 100% (93-164) | 97% / 112% | 2.84 | 2 | 12.7 s | 0 |
| obstacle_field / smart | 8% (1-27) | 62% (31-74) | 13% / 59% | 2.95 | 5 | 6.8 s | 0 |
| open / reactive | 0% (0-22) | 57% (0-82) | 7% / 67% | 2.96 | 3 | 6.2 s | 1 run |
| open / adaptive | 86% (80-88) | 88% (85-90) | 85% / 88% | 3.00 | 0 | - | 0 |
| open / smart | 0% (0-21) | 1% (0-38) | 2% / 4% | 3.07 | 3 | 7.1 s | 0 |

## Reading

- Wall + obstacle contact: smart is lower than reactive in both arenas and
  both seed sets (near band roughly halved in obstacle_field, about 0-4% vs
  60-70% in the open arena). Holdout agrees with main.
- Survival: **the "capture time not shorter than reactive" goal is not met in
  obstacle_field** (smart is caught sooner: 3.3 s vs 8.4 s on main, 6.8 s vs
  12.0 s on holdout). In the open arena smart and reactive are comparable
  (7 vs 5 captures of 10, 3 vs 3 of 5; capture times 6.2 vs 5.6 s and 7.1 vs
  6.2 s). With n this small only the obstacle_field gap looks consistent.
- Inference (not tested separately): capture is "target inside the hull of >= 3
  pursuers within 2.5", so a target pressed against a wall cannot be enclosed.
  In `sw3` (an earlier sweep, not kept) the surviving reactive runs in the
  open arena were the high-wall-fraction ones. Wall-hugging is therefore a
  genuine survival tactic against this pursuit, which is in tension with the
  "do not hug walls" goal. `smart` trades some of that protection for open
  movement and counters it with an enclosure-risk term.
- `adaptive` sits on the wall 85-90% of the time.
- Stuck (>= 2 s below 0.5 m/s): none for smart; one reactive run on holdout.

Files: `summary.json` (both sets). Per-run `main/*.json` and `holdout/*.json`
are git-ignored raw data per `artifacts/README.md`; regenerate with
`tools/evader_compare.py sweep` and `summarize`.
