"""Procedural, seed-deterministic environments (SDD v4 Part C, M13).

Environments must be *strategically meaningful* — break the perimeter loop
and create chokepoints, not just decorate. Every layout derives entirely
from the seed so strategy benchmarks stay fair (v4 C.5). No time/random
globals — only a seeded random.Random, so runs reproduce exactly.

A Layout carries:
- obstacles: [(x, y, r)] circles — reuse the v3 boid V_obs + collision +
  the v4 occlusion ray-casts (all already circle-based).
- zones: [(x, y, r, kind)] — 'capture' (win on entry) or 'slow' (tar pit:
  nullifies the target's 2× speed). New sim feature (M13).
- shrink_rate: >0 ⇒ battle-royale bounds contraction (C.4), which makes
  perimeter-circling physically impossible and guarantees termination.
"""

import math
from dataclasses import dataclass, field


@dataclass
class Layout:
    obstacles: list = field(default_factory=list)   # (x, y, r)
    zones: list = field(default_factory=list)       # (x, y, r, kind)
    shrink_rate: float = 0.0                         # units/s of bounds inset
    env_type: str = 'open'

    def obstacles_flat(self):
        flat = []
        for (x, y, r) in self.obstacles:
            flat += [x, y, r]
        return flat if flat else [0.0]

    def zones_flat(self):
        kinds = {'capture': 0.0, 'slow': 1.0}
        flat = []
        for (x, y, r, kind) in self.zones:
            flat += [x, y, r, kinds.get(kind, 0.0)]
        return flat if flat else [0.0]


class WorldGenerator:
    """Deterministic layout factory. Same (seed, world_size, env_type) ⇒
    byte-identical layout."""

    def __init__(self, seed, world_size=20.0):
        self.rng = __import__('random').Random(seed)
        self.w = world_size

    # -- Poisson-ish rejection sampling (deterministic) --------------------
    def _scatter(self, n, r_range, margin, min_gap, existing):
        placed = list(existing)
        out = []
        tries = 0
        while len(out) < n and tries < n * 60:
            tries += 1
            x = self.rng.uniform(margin, self.w - margin)
            y = self.rng.uniform(margin, self.w - margin)
            r = self.rng.uniform(*r_range)
            if all(math.hypot(x - px, y - py) > r + pr + min_gap
                   for (px, py, pr) in placed):
                out.append((x, y, r))
                placed.append((x, y, r))
        return out

    # -- layouts -----------------------------------------------------------
    def generate(self, env_type='open'):
        if env_type == 'open':
            return Layout(env_type='open')
        if env_type == 'obstacle_field':
            return self._obstacle_field()
        if env_type == 'pillar':
            return self._pillar()
        if env_type == 'zones':
            return self._zones()
        if env_type == 'shrink':
            # fast enough that the box reaches the tight final extent well
            # inside a typical episode, guaranteeing termination (C.4)
            return Layout(shrink_rate=0.14, env_type='shrink')
        raise ValueError(f'unknown env_type: {env_type}')

    # FIXED obstacle_field layout (world coords for a 20x20 arena, y-up).
    # Hand-placed to match the chosen map exactly: dense, varied sizes,
    # obstacles reaching the corners/edges, with a clear passage between
    # every pair (a navigable maze, no dead-end pockets). Seed-independent,
    # so the map is identical every run.
    FIXED_FIELD = [
        # big obstacles (some cut off at the corners/edges)
        # top-centre pulled down off the top wall -> a lane opens above it
        (2.4, 17.3, 1.8), (9.6, 16.7, 2.1), (16.9, 17.3, 2.2),
        (18.1, 12.0, 2.5), (4.9, 11.0, 2.5),
        (8.0, 6.0, 2.6), (1.3, 2.7, 2.8), (17.2, 3.1, 2.6),
        # small fillers
        (2.6, 14.2, 0.8), (9.9, 11.3, 1.1),
        (13.5, 6.1, 0.9), (10.9, 1.9, 0.9),
    ]

    def _obstacle_field(self):
        """The fixed, hand-authored obstacle maze (C.1). Same every run —
        does not depend on the seed."""
        scale = self.w / 20.0            # allow non-20 arenas to scale
        obs = [(x * scale, y * scale, r * scale)
               for (x, y, r) in self.FIXED_FIELD]
        return Layout(obstacles=obs, env_type='obstacle_field')

    def _pillar(self):
        """Central pillar: the target can still circle it, but a *smaller*
        loop is far easier to blockade (C.3)."""
        r = self.rng.uniform(2.0, 3.0)
        return Layout(obstacles=[(self.w * 0.5, self.w * 0.5, r)],
                      env_type='pillar')

    def _zones(self):
        """Capture zones / tar pits — herd the target in for a direct win
        (C.2). One capture zone in a seed-chosen corner, one slow pit."""
        m = 4.0
        corners = [(m, m), (m, self.w - m), (self.w - m, m),
                   (self.w - m, self.w - m)]
        cap = self.rng.choice(corners)
        remaining = [c for c in corners if c != cap]
        pit = self.rng.choice(remaining)
        return Layout(
            zones=[(cap[0], cap[1], 2.2, 'capture'),
                   (pit[0], pit[1], 2.0, 'slow')],
            env_type='zones')
