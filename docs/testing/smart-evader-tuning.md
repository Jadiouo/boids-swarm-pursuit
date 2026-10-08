# smart evader: tuning log (2026-10-08)

Not an experiment. Every attempt that was run is listed, including those
that did not help. Sim rules and pursuer parameters were never changed; only
`SmartConfig` fields of the smart evader.

Measurement: `tools/evader_compare.py` (12 agents, perfect perception, legacy
sharing, 30 s, 4x; episode 2 of 2). Columns: wall+obstacle touch median %,
near median %, mean speed, captures of 10 / median capture time (s). Seeds
1-10 unless noted; machine load average 50-85 throughout, so single numbers
are noisy (open-arena captures move by +-2-3 between identical reruns).

## Measurement mistakes found on the way

1. First sweeps used one episode per run. The sim clock starts before the
   controllers finish importing, so the target stood still for seconds at 4x
   and mean speed was 0.66; those numbers (and the first "smart is caught in
   3 s" reading) were an artifact. Fixed by recording episode 2 of 2.
2. `rec.py` of the earlier diagnostic used a 0.7 body radius for "touch";
   the sim's target radius is 0.15 (params.yaml). `touch` now uses 0.15+0.03,
   and the old 0.73 band is kept as `near`.

## Attempts (smart only)

| id | change | obstacle_field touch / near / speed / caps / t | open touch / near / speed / caps / t |
|---|---|---|---|
| V0 | geodesic gradient path + pursuer repulsion + wall tangent filter, no rollouts | 16 / 66 / 1.63 / 10 / 3.1 | 0 / 3 / 1.68 / 7 / 3.2 |
| V1 | control layer replaced by turn-rate-limited arc rollouts (heading aware) | 16 / 63 / 2.85 / 10 / 4.4 | 0 / 0 / 3.16 / 9 / 4.9 |
| V2 | + internal stamina estimate (rollout speed = 2.0 when tank empty) | 15 / 64 / 2.92 / 10 / 5.9 | 0 / 1 / 3.03 / 6 / 5.0 |
| A | danger radius 3.5, w_danger 2.5 | 17 / 83 / 2.67 / 10 / 4.5 | 0 / 2 / 3.07 / 5 / 5.0 |
| B | horizon 3.0 s | 5 / 62 / 2.82 / 10 / 3.6 | 0 / 0 / 3.01 / 8 / 4.9 (9 valid) |
| C | w_clear 0.6 | 6 / 55 / 2.91 / 10 / 3.7 | 0 / 4 / 3.13 / 8 / 4.7 |
| D | enclosure-risk term, w_enclose 1.5 | 14 / 65 / 2.84 / 10 / 4.0 | 0 / 0 / 3.13 / 9 / 5.2 |
| E | w_enclose 3, horizon 3 | 13 / 68 / 2.83 / 10 / 5.1 | 0 / 0 / 3.17 / 9 / 5.1 (9 valid) |
| F | w_open 0, w_clear 0.05, w_enclose 3, w_dead 0.3 | 27 / 68 / 2.68 / 10 / 3.3 | 0 / 4 / 3.07 / 5 / 5.8 |
| G | F with w_clear 0.3 (final) | 15 / 61 / 2.80 / 10 / 3.2 | 0 / 3 / 3.16 / 7 / 4.9 |

Second seed set (21-30, not used for the final or the holdout), D vs F vs G:

| id | obstacle_field | open |
|---|---|---|
| D | 21 / 69 / 2.88 / 10 / 8.0 | 0 / 0 / 3.03 / 6 / 4.7 |
| F | 17 / 61 / 2.98 / 10 / 5.6 | 0 / 2 / 3.00 / 3 / 4.8 |
| G | 15 / 53 / 2.92 / 10 / 5.7 | 0 / 2 / 3.05 / 4 / 5.5 |

## Why the final weights

The open arena exposed the real trade-off. The game captures by hull
containment, so staying in the open centre loses to the pack (V1-E: 5-9 of
10 caught, near ~0%), while reactive survives by sitting on a wall. The
enclosure-risk term (how small the widest pursuer-free arc around the
predicted position is) plus de-emphasising "openness" lets smart stay off
walls and still survive about as often as reactive in the open arena (G:
11 of 20 caught over two seed sets; F: 8 of 20; D: 15 of 20). G was picked
over F for lower obstacle contact (15 vs 17-27). The F/G difference in open
captures is within noise.

The obstacle_field capture time stayed shorter than reactive in every
variant. No attempt closed that gap; it is reported as such.

## Final config, validation

Defaults in `SmartConfig` = G. Final comparison on seeds 1-10 (not
independent: it overlaps the tuning seeds) and once on holdout seeds 11-15:
`artifacts/evader-compare-2026-10-08/`.

## compute() time

Standalone, 12 random-walking pursuers, 4500 calls per arena, load average
~50 on the machine: obstacle_field p50 0.36 ms, p95 1.07 ms, p99 1.34 ms,
max 15 ms (one outlier); open p50 0.35 ms, p95 1.67 ms, max 5.6 ms. The
goal decision (3 geodesic-field phases at 5 Hz) is spread over consecutive
calls so no call pays for more than one field; a new goal field is built
only when the goal changes. The unit test asserts p95 < 5 ms (best of 3
trials).
