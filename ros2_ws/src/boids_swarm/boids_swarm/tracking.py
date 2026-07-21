"""Controller-side track filter + data association (SDD v4 A.5).

Raw detections are noisy and drop out. A short-lived track per detected
neighbor smooths the noise, bridges gaps, and — crucially — *derives*
velocity (hence heading & speed) that a range/bearing sensor never gives
directly. Tracked estimates, not raw blips, feed the steering behaviors;
this also protects the P-controller from the ±π chatter that noisy input
caused in v3 M2 (bug #2).

Filter: constant-velocity alpha-beta (a 2-gain scalar Kalman). Association:
by reported id when the sensor emits ids, else nearest-neighbour within a
gate (the identity-ambiguity case, v4 A.3).

Pure math, no ROS — unit-tested (v4 E.2).
"""

import math


class Track:
    __slots__ = ('x', 'y', 'vx', 'vy', 't', 'tid', 'is_target',
                 'hits', 'last_seen')

    def __init__(self, x, y, t, tid, is_target=False):
        self.x, self.y = x, y
        self.vx = self.vy = 0.0
        self.t = t
        self.tid = tid
        self.is_target = is_target
        self.hits = 1
        self.last_seen = t

    def predict(self, t):
        dt = max(t - self.t, 0.0)
        return self.x + self.vx * dt, self.y + self.vy * dt

    def update(self, zx, zy, t, alpha, beta):
        dt = t - self.t
        if dt <= 1e-6:
            self.x, self.y = zx, zy
            self.last_seen = t
            self.hits += 1
            return
        px, py = self.x + self.vx * dt, self.y + self.vy * dt
        rx, ry = zx - px, zy - py                 # residual
        self.x = px + alpha * rx
        self.y = py + alpha * ry
        self.vx += (beta / dt) * rx
        self.vy += (beta / dt) * ry
        self.t = t
        self.last_seen = t
        self.hits += 1

    @property
    def heading(self):
        return math.atan2(self.vy, self.vx)

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)

    @property
    def confident(self):
        # need a couple of hits before velocity is meaningful
        return self.hits >= 3


class Tracker:
    """Maintains tracks across frames. `step` returns the live tracks."""

    def __init__(self, alpha=0.5, beta=0.12, gate=1.8, max_age=0.6):
        self.alpha = alpha
        self.beta = beta
        self.gate = gate                          # NN association radius
        self.max_age = max_age                    # drop after unseen this long
        self.tracks: dict = {}                    # key -> Track
        self._next_key = 0

    def step(self, dets, t):
        """dets: list of (x, y, id, is_target). id < 0 ⇒ associate by NN.

        Returns (neighbor_tracks, target_track_or_None).
        """
        id_dets = [(x, y, i, tg) for (x, y, i, tg) in dets if i >= 0]
        anon_dets = [(x, y, i, tg) for (x, y, i, tg) in dets if i < 0]
        matched = set()

        # 1) id-based association (exact)
        for (x, y, i, tg) in id_dets:
            key = ('id', int(i))
            if key in self.tracks:
                self.tracks[key].update(x, y, t, self.alpha, self.beta)
            else:
                self.tracks[key] = Track(x, y, t, int(i), bool(tg))
            matched.add(key)

        # 2) nearest-neighbour association for anonymous blips
        anon_keys = [k for k in self.tracks if k[0] == 'anon']
        for (x, y, _i, tg) in anon_dets:
            best, best_d = None, self.gate
            for k in anon_keys:
                if k in matched:
                    continue
                px, py = self.tracks[k].predict(t)
                d = math.hypot(px - x, py - y)
                if d < best_d:
                    best, best_d = k, d
            if best is None:
                best = ('anon', self._next_key)
                self._next_key += 1
                self.tracks[best] = Track(x, y, t, -1, bool(tg))
            else:
                self.tracks[best].update(x, y, t, self.alpha, self.beta)
            matched.add(best)

        # 3) prune stale tracks
        for k in [k for k, tr in self.tracks.items()
                  if t - tr.last_seen > self.max_age]:
            del self.tracks[k]

        neighbors = [tr for tr in self.tracks.values() if not tr.is_target]
        targets = [tr for tr in self.tracks.values() if tr.is_target]
        # keep the freshest target track if several (id churn under dropout)
        target = max(targets, key=lambda tr: tr.last_seen) if targets else None
        return neighbors, target


class CirclingDetector:
    """Detect a target hugging the perimeter loop (SDD v4 B.1 trigger).

    Circling = the target's angle about the arena centre advances with a
    consistent sign for `sustain` seconds while it stays near the wall.
    Runs on the shared/tracked target estimate, so every boid computes the
    same trigger from its own belief — no central coordinator.
    """

    def __init__(self, window=2.5, sustain=1.5, speed_thresh=0.35,
                 perimeter_frac=0.45):
        self.window = window
        self.sustain = sustain
        self.speed_thresh = speed_thresh
        self.perimeter_frac = perimeter_frac
        self.hist = []                 # (t, unwrapped_angle, near_wall)
        self._unwrapped = None
        self._prev_ang = None
        self._circ_since = None

    def update(self, t, target_xy, center, half_extent):
        ang = math.atan2(target_xy[1] - center[1],
                         target_xy[0] - center[0])
        if self._prev_ang is None:
            self._unwrapped = ang
        else:
            d = math.atan2(math.sin(ang - self._prev_ang),
                           math.cos(ang - self._prev_ang))
            self._unwrapped += d
        self._prev_ang = ang
        r = math.hypot(target_xy[0] - center[0], target_xy[1] - center[1])
        near_wall = r > self.perimeter_frac * half_extent
        self.hist.append((t, self._unwrapped, near_wall))
        self.hist = [h for h in self.hist if t - h[0] <= self.window]

        circling, circ_dir = False, 0.0
        if len(self.hist) >= 3 and (self.hist[-1][0] - self.hist[0][0]) > 0.3:
            dt = self.hist[-1][0] - self.hist[0][0]
            ang_vel = (self.hist[-1][1] - self.hist[0][1]) / dt
            frac_wall = sum(1 for h in self.hist if h[2]) / len(self.hist)
            if abs(ang_vel) > self.speed_thresh and frac_wall > 0.6:
                if self._circ_since is None:
                    self._circ_since = t
                if t - self._circ_since >= self.sustain:
                    circling = True
                    circ_dir = 1.0 if ang_vel > 0 else -1.0
            else:
                self._circ_since = None
        return circling, circ_dir, r
