"""Pure statistics helpers for the phase-2 relay experiment (no ROS)."""

import math


def wilson(k, n, z=1.96):
    """Wilson score interval (default 95%) as (lo, hi); None when n == 0."""
    if n == 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def median(values):
    vals = sorted(values)
    if not vals:
        return None
    m = len(vals) // 2
    return vals[m] if len(vals) % 2 else (vals[m - 1] + vals[m]) / 2


def expected_in_range_counts(events, last_status_stamp, margin=0.05):
    """Per-receiver count of published sightings that had the receiver in
    radio range, restricted to sightings published before the receiver's
    last reported status (minus margin) so counters and denominator cover
    the same time window.

    events: [{'t': sim_time, 'exp': [receiver ids]}, ...]
    last_status_stamp: {receiver: sim time of its last status message}
    """
    counts = {r: 0 for r in last_status_stamp}
    for ev in events:
        for r in ev['exp']:
            cut = last_status_stamp.get(r)
            if cut is not None and ev['t'] <= cut - margin:
                counts[r] += 1
    return counts


def per_receiver_drop(expected, received):
    """1 - received/expected per receiver with expected > 0."""
    return {r: 1.0 - received.get(r, 0) / e
            for r, e in expected.items() if e > 0}


def pooled_drop(expected, received):
    """1 - sum(received)/sum(expected) over receivers with expected > 0."""
    e = sum(v for v in expected.values() if v > 0)
    if e == 0:
        return None
    r = sum(received.get(k, 0) for k, v in expected.items() if v > 0)
    return 1.0 - r / e


def km_curve(times, observed, z=1.96):
    """Kaplan-Meier survival (time to capture) with Greenwood 95% band.

    times: follow-up time per run (capture time, or the right-censoring time
    for runs that ended without capture); observed: bool per run, True if the
    capture was observed. Returns [(t, S, lo, hi)] at each distinct event
    time, starting with (0, 1, 1, 1). Band = plain Greenwood, clipped to
    [0, 1] (exploratory; wide and not exact at n = 20).
    """
    data = sorted(zip(times, observed))
    at_risk = len(data)
    out = [(0.0, 1.0, 1.0, 1.0)]
    s, gsum = 1.0, 0.0
    i = 0
    while i < len(data):
        t = data[i][0]
        d = c = 0
        while i < len(data) and data[i][0] == t:
            d += bool(data[i][1])
            c += not data[i][1]
            i += 1
        if d:
            s *= 1.0 - d / at_risk
            if at_risk > d:
                gsum += d / (at_risk * (at_risk - d))
            se = s * math.sqrt(gsum)
            out.append((t, s, max(0.0, s - z * se), min(1.0, s + z * se)))
        at_risk -= d + c
    return out


def km_at(curve, t):
    """Step-function value (S, lo, hi) of a km_curve at time t."""
    cur = curve[0]
    for row in curve:
        if row[0] <= t:
            cur = row
        else:
            break
    return cur[1:]


def bootstrap_pooled_drop_ci(run_pairs, n_boot=10000, seed=0, alpha=0.05):
    """Run-level percentile bootstrap CI of the pooled drop rate.

    run_pairs: [(expected_total, received_total)] one per run (the run is
    the resampling unit; pooled drop = 1 - sum(received)/sum(expected)).
    Returns (point, lo, hi) or None when there is no expected traffic.
    """
    import random
    pairs = [(e, r) for e, r in run_pairs if e > 0]
    if not pairs:
        return None
    rnd = random.Random(seed)
    n = len(pairs)
    pt = 1.0 - sum(r for _, r in pairs) / sum(e for e, _ in pairs)
    vals = []
    for _ in range(n_boot):
        sample = [pairs[rnd.randrange(n)] for _ in range(n)]
        e = sum(x[0] for x in sample)
        vals.append(1.0 - sum(x[1] for x in sample) / e)
    vals.sort()
    lo = vals[int((alpha / 2) * n_boot)]
    hi = vals[min(n_boot - 1, int((1 - alpha / 2) * n_boot))]
    return pt, lo, hi


def n_per_group_two_proportions(p1, p2, alpha=0.05, power=0.80,
                                continuity=False):
    """Runs per cell for a two-sided two-proportion z test to detect
    p1 vs p2 (equal group sizes). continuity=True applies the Fleiss
    continuity correction. Returns a float (callers round up)."""
    from statistics import NormalDist
    nd = NormalDist()
    za, zb = nd.inv_cdf(1 - alpha / 2), nd.inv_cdf(power)
    pb = (p1 + p2) / 2
    d = abs(p1 - p2)
    n = (za * math.sqrt(2 * pb * (1 - pb))
         + zb * math.sqrt(p1 * (1 - p1) + p2 * (1 - p2))) ** 2 / d ** 2
    if continuity:
        n = n / 4 * (1 + math.sqrt(1 + 4 / (n * d))) ** 2
    return n


def window_fraction(rows, t_end):
    """Fraction of cycle rows with belief_valid among own-pose-fresh cycle
    rows whose stamp lies in [t0, t0 + t_end], t0 = first cycle stamp.
    Returns (fraction, denominator) or (None, 0)."""
    cyc = [s for s in rows if s.get('event') == 'cycle' and s.get('own_pose_fresh')]
    if not cyc:
        return None, 0
    t0 = min(s['stamp'] for s in cyc)
    w = [s for s in cyc if s['stamp'] <= t0 + t_end]
    if not w:
        return None, 0
    return sum(bool(s['belief_valid']) for s in w) / len(w), len(w)
