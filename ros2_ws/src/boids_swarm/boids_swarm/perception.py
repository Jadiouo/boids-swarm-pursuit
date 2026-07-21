"""Sim-side per-agent sensor synthesis (SDD v4 Part A).

The sim is the sole world-owner, so it computes *what each agent can
actually detect* — FOV cone, occlusion, distance-growing noise, dropout —
and publishes only that as a relative range/bearing list. This replaces
global ground-truth for the boids (kept behind `perception_mode`).

Pure math, no ROS imports, so it is unit-testable (v4 E.2). O(N²) per
frame is trivial for N≈12–20; vectorize with numpy only if N grows large.

Detection stride (6 floats), all in the OBSERVER's frame:
    [range, bearing, rel_heading, speed, is_target, id]
- range, bearing: measured (noisy) polar position; bearing is relative to
  the observer's heading.
- rel_heading: neighbor heading minus observer heading. A range/bearing-
  only sensor would not give this; we include it as a coarse observed
  heading (visual pose) so alignment/pursuit stay functional pre-tracking.
  Tracking (M9) later derives heading from track velocity instead.
- speed: observed scalar speed (target pursuit needs it; refined by M9).
- is_target: 1.0 for the evader, else 0.0.
- id: candidate index, or -1.0 when `emit_ids` is off (M9 must associate).
"""

import math

from .geometry import wrap_angle

STRIDE = 6


def _ray_blocked(ox, oy, ux, uy, target_range, occluders):
    """True if a circle occluder crosses the observer→candidate segment
    strictly before the candidate. (ux, uy) is the unit ray direction.

    The observer's own body sits at projection≈0 and the candidate's at
    ≈target_range; both are excluded by the projection guards, so they
    never self-occlude.
    """
    for (cx, cy, r) in occluders:
        fx, fy = cx - ox, cy - oy
        proj = fx * ux + fy * uy
        if proj <= 1e-3 or proj >= target_range - 1e-3:
            continue                       # behind observer / at candidate
        perp2 = (fx * fx + fy * fy) - proj * proj
        if perp2 < r * r:
            return True
    return False


def compute_detections(observer, candidates, occluders, cfg, rng):
    """Return the observer's detection list.

    observer:   (x, y, theta)
    candidates: iterable of (id, x, y, theta, speed, is_target)
    occluders:  iterable of (x, y, radius) — obstacles + agent/target bodies
    cfg:        dict with fov, sensor_range, occlusion, range_sigma,
                bearing_sigma, p_miss, emit_ids
    rng:        seeded random.Random (deterministic benchmarks)
    """
    ox, oy, oth = observer
    fov = cfg['fov']
    srange = cfg['sensor_range']
    dets = []
    for (cid, cx, cy, cth, cspeed, is_target) in candidates:
        dx, dy = cx - ox, cy - oy
        r_true = math.hypot(dx, dy)
        if r_true < 1e-6 or r_true > srange:
            continue                       # out of range (or self)
        bearing = wrap_angle(math.atan2(dy, dx) - oth)
        if abs(bearing) > fov:
            continue                       # outside the FOV cone
        if cfg['occlusion']:
            if _ray_blocked(ox, oy, dx / r_true, dy / r_true, r_true,
                            occluders):
                continue                   # hidden behind body/obstacle
        # dropout, rising with range
        if rng.random() < cfg['p_miss'] * (r_true / srange):
            continue
        # measurement noise, growing with distance (far = noisier)
        r_sigma = cfg['range_sigma'] * r_true
        b_sigma = cfg['bearing_sigma'] * (0.5 + 0.5 * r_true / srange)
        m_range = max(r_true + rng.gauss(0.0, r_sigma), 0.01)
        m_bearing = wrap_angle(bearing + rng.gauss(0.0, b_sigma))
        rel_head = wrap_angle(cth - oth)
        det_id = float(cid) if cfg['emit_ids'] else -1.0
        dets.append((m_range, m_bearing, rel_head, cspeed,
                     1.0 if is_target else 0.0, det_id))
    return dets


def relayed_detection(observer, target, cid, jitter, rng):
    """Build a target detection for a boid that heard the sighting over
    comms rather than seeing it (v4 A.6). Expressed in the receiver's own
    frame, with a little relay jitter but no FOV/occlusion/dropout gating.

    observer: (x, y, theta); target: (x, y, theta, speed)
    """
    ox, oy, oth = observer
    tx, ty, tth, tspeed = target
    dx, dy = tx - ox, ty - oy
    r = max(math.hypot(dx, dy) + rng.gauss(0.0, jitter), 0.01)
    bearing = wrap_angle(math.atan2(dy, dx) - oth + rng.gauss(0.0, jitter))
    rel_head = wrap_angle(tth - oth)
    return (r, bearing, rel_head, tspeed, 1.0, float(cid))


def detections_to_flat(dets):
    """Pack a detection list into [K, d0..dK] for Float32MultiArray."""
    flat = [float(len(dets))]
    for d in dets:
        flat.extend(float(v) for v in d)
    return flat


def flat_to_world(flat, observer):
    """Controller-side inverse: relative detections → world estimates.

    Returns (neighbors, target) where neighbors is a list of
    (x, y, theta, id) and target is (x, y, theta, speed, id) or None.
    Uses the observer's own pose (odometry) to place each blip.
    """
    ox, oy, oth = observer
    k = int(flat[0]) if flat else 0
    neighbors, target = [], None
    for j in range(k):
        b = 1 + STRIDE * j
        rng_, bear, rel_head, speed, is_tgt, cid = flat[b:b + STRIDE]
        ang = oth + bear
        wx, wy = ox + rng_ * math.cos(ang), oy + rng_ * math.sin(ang)
        wth = wrap_angle(oth + rel_head)
        if is_tgt >= 0.5:
            target = (wx, wy, wth, speed, cid)
        else:
            neighbors.append((wx, wy, wth, cid))
    return neighbors, target
