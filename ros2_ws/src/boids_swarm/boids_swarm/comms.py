"""Information sharing via a range-limited mesh (SDD v4 A.6, M10).

This is the moment the individual-vs-swarm distinction becomes real: a
boid that sees the target broadcasts the sighting, peers relay it over
short-range links, and the *group* ends up tracking a target most of its
*individuals* cannot see.

Modelled as another sim-mediated "sensor": the sim (world-owner) knows
every position, so it builds the comm graph (edge between i, j when
|p_i − p_j| <= comm_range) and propagates each sighting across connected
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
            if (xi - xj) ** 2 + (yi - yj) ** 2 <= r2:
                dsu.union(i, j)
    return dsu


def _within_hops(positions, comm_range, seers, max_hops):
    """Multi-source BFS over the '<=' comm graph, at most max_hops edges."""
    n = len(positions)
    r2 = comm_range * comm_range
    informed = list(seers)
    frontier = [i for i, s in enumerate(seers) if s]
    for _ in range(max_hops):
        nxt = []
        for i in frontier:
            xi, yi = positions[i]
            for j in range(n):
                if informed[j]:
                    continue
                xj, yj = positions[j]
                if (xi - xj) ** 2 + (yi - yj) ** 2 <= r2:
                    informed[j] = True
                    nxt.append(j)
        frontier = nxt
        if not frontier:
            break
    return informed


def propagate_sightings(positions, comm_range, seers, max_hops=0):
    """Which agents know the target after mesh relay.

    positions: [(x, y), ...]; seers: bool list — who *directly* sees it.
    Returns a bool list: informed[i] is True if agent i sees it directly
    OR is reachable from someone who does within max_hops comm edges.
    max_hops=0 means unlimited (whole connected component, phase 1
    behavior); max_hops=1 is a single radio hop.
    """
    n = len(positions)
    if not any(seers):
        return [False] * n
    if max_hops and max_hops > 0:
        return _within_hops(positions, comm_range, seers, int(max_hops))
    dsu = comm_components(positions, comm_range)
    informed_roots = {dsu.find(i) for i, s in enumerate(seers) if s}
    return [dsu.find(i) in informed_roots for i in range(n)]


def receivers_in_range(positions, sender, radio_range):
    """Agent indices (excluding the sender) within radio_range of it.

    Inclusive bound ('<=', same as the comm graph), matching the
    receiver-side gate in the controller (`distance > radio_range` is out
    of range). Sim-side denominator for
    per-receiver loss.
    """
    sx, sy = positions[sender]
    r2 = radio_range * radio_range
    return [j for j, (x, y) in enumerate(positions)
            if j != sender and (x - sx) ** 2 + (y - sy) ** 2 <= r2]
