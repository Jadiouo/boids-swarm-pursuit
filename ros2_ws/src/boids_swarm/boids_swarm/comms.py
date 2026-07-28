"""Information sharing via a range-limited mesh (SDD v4 A.6, M10).

This is the moment the individual-vs-swarm distinction becomes real: a
boid that sees the target broadcasts the sighting, peers relay it over
short-range links, and the *group* ends up tracking a target most of its
*individuals* cannot see.

Modelled as another sim-mediated "sensor": the sim (world-owner) knows
every position, so it builds the comm graph (edge between i, j when
|p_i − p_j| < comm_range) and propagates each sighting across connected
components. Turning comms off leaves only the boids that directly see the
target reacting — which is exactly how you *show* the gap.

Pure math, no ROS — unit-tested (v4 E.2).
"""


class _DSU:
    """Union-find for connected components of the comm graph."""

    def __init__(self, n):
        self.parent = list(range(n))

    def find(self, a):
        while self.parent[a] != a:
            self.parent[a] = self.parent[self.parent[a]]
            a = self.parent[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def comm_components(positions, comm_range):
    """Return a DSU over agents linked when within comm_range.

    O(N²) edge scan — trivial for N≈12–20 (vectorize only if N grows).
    """
    n = len(positions)
    dsu = _DSU(n)
    r2 = comm_range * comm_range
    for i in range(n):
        xi, yi = positions[i]
        for j in range(i + 1, n):
            xj, yj = positions[j]
            if (xi - xj) ** 2 + (yi - yj) ** 2 < r2:
                dsu.union(i, j)
    return dsu


def propagate_sightings(positions, comm_range, seers):
    """Which agents know the target after mesh relay.

    positions: [(x, y), ...]; seers: bool list — who *directly* sees it.
    Returns a bool list: informed[i] is True if agent i sees it directly
    OR shares a connected component with someone who does.
    """
    n = len(positions)
    if not any(seers):
        return [False] * n
    dsu = comm_components(positions, comm_range)
    informed_roots = {dsu.find(i) for i, s in enumerate(seers) if s}
    return [dsu.find(i) in informed_roots for i in range(n)]
