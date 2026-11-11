"""Collision-checked polyline shortcuts on a footprint-inflated observed grid.

This is geometric shortening, not curvature smoothing or a flyable trajectory.
Remaining hard corners require a stop or future collision-checked fillets.
"""

from math import ceil, cos, floor, hypot, isfinite, sin

from dronebench.grid import blocked_cells


class LineOfSight:
    """Cache inflation for one immutable OccupancySlice and vehicle radius.

    Closed cell boundaries count as occupied when either touching cell is
    blocked. Unknown and out-of-map space are blocked. Coordinates are metres
    in the same frame as the grid; grid origin rotation is respected.
    """

    def __init__(self, grid, radius, occupied_threshold=50):
        self.grid = grid
        self.blocked = blocked_cells(grid, radius, occupied_threshold)

    def _local(self, point):
        if len(point) != 2 or not all(isfinite(v) for v in point):
            raise ValueError("finite XY point required")
        dx, dy = point[0] - self.grid.origin_x, point[1] - self.grid.origin_y
        c, s = cos(self.grid.origin_yaw), sin(self.grid.origin_yaw)
        return ((c * dx + s * dy) / self.grid.resolution,
                (-s * dx + c * dy) / self.grid.resolution)

    def clear(self, start, end):
        a, b = self._local(start), self._local(end)
        eps = 1e-9  # grid cells; conservatively includes nearby boundaries
        for x, y in (a, b):
            if not (eps < x < self.grid.width - eps and eps < y < self.grid.height - eps):
                return False
        delta = b[0] - a[0], b[1] - a[1]
        cuts = {0., 1.}
        for axis in (0, 1):
            if abs(delta[axis]) <= eps:
                continue
            for boundary in range(floor(min(a[axis], b[axis])) + 1,
                                  ceil(max(a[axis], b[axis]))):
                t = (boundary - a[axis]) / delta[axis]
                if 0 < t < 1:
                    cuts.add(t)
        cuts = sorted(cuts)
        # Inspect both crossing points and open segment interiors. At a vertex
        # all four touching cells are checked; along an edge both sides are.
        samples = cuts + [(lo + hi) / 2 for lo, hi in zip(cuts, cuts[1:])]
        for t in samples:
            x, y = a[0] + t * delta[0], a[1] + t * delta[1]
            cols = (round(x) - 1, round(x)) if abs(x - round(x)) <= eps else (floor(x),)
            rows = (round(y) - 1, round(y)) if abs(y - round(y)) <= eps else (floor(y),)
            for col in cols:
                for row in rows:
                    if (not 0 <= col < self.grid.width or not 0 <= row < self.grid.height
                            or (col, row) in self.blocked):
                        return False
        return True


def shortcut_path(grid, path, radius, occupied_threshold=50, *, start=None, goal=None):
    """Greedy farthest-visible subsequence, preserving checked endpoints.

    Arbitrary start/goal connectors are allowed only when the resulting original
    segments are clear. Invalid original paths raise ValueError, so shortcuts
    cannot silently repair unvalidated input. No global shortest-path claim.
    """
    points = [tuple(point) for point in path]
    if not points:
        raise ValueError("nonempty path required")
    if start is not None and tuple(start) != points[0]:
        points.insert(0, tuple(start))
    if goal is not None and tuple(goal) != points[-1]:
        points.append(tuple(goal))
    checker = LineOfSight(grid, radius, occupied_threshold)
    if any(not checker.clear(point, point) for point in points):
        raise ValueError("path endpoint touches blocked or out-of-map space")
    if any(not checker.clear(a, b) for a, b in zip(points, points[1:])):
        raise ValueError("input path or connector crosses blocked space")
    result, index = [points[0]], 0
    while index < len(points) - 1:
        for candidate in range(len(points) - 1, index, -1):
            if checker.clear(points[index], points[candidate]):
                result.append(points[candidate])
                index = candidate
                break
    return result


def path_length(path):
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
