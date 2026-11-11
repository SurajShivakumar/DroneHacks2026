"""Conservative A* over an observed occupancy slice, not simulator geometry.

The caller supplies a fresh ROS-style grid in a calibrated ENU map frame and
chooses a height slice clear for the full aircraft envelope. Unknown cells are
blocked. This module neither estimates pose nor constructs a map from truth.
It is a path component, not a complete 3D tunnel/window/landing planner.
"""

from dataclasses import dataclass
import heapq
from math import ceil, cos, floor, hypot, isfinite, sin, sqrt


@dataclass(frozen=True)
class OccupancySlice:
    width: int
    height: int
    resolution: float
    cells: tuple
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def __post_init__(self):
        if type(self.width) is not int or type(self.height) is not int or min(self.width, self.height) < 1:
            raise ValueError("grid dimensions must be positive integers")
        if len(self.cells) != self.width * self.height:
            raise ValueError("grid dimensions do not match cells")
        if not isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("resolution must be finite and positive")
        if not all(isfinite(x) for x in (self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError("grid transform must be finite")
        if any(type(x) is not int or not -1 <= x <= 100 for x in self.cells):
            raise ValueError("occupancy must be integer -1 (unknown) or 0..100")

    def world_to_cell(self, x, y):
        if not isfinite(x) or not isfinite(y):
            raise ValueError("position must be finite")
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = cos(self.origin_yaw), sin(self.origin_yaw)
        return floor((c * dx + s * dy) / self.resolution), floor((-s * dx + c * dy) / self.resolution)

    def cell_to_world(self, col, row):
        x, y = (col + .5) * self.resolution, (row + .5) * self.resolution
        c, s = cos(self.origin_yaw), sin(self.origin_yaw)
        return self.origin_x + c * x - s * y, self.origin_y + s * x + c * y


def blocked_cells(grid, radius, occupied_threshold=50):
    if not isfinite(radius) or radius < 0:
        raise ValueError("radius must include vehicle extent and uncertainty")
    if type(occupied_threshold) is not int or not 1 <= occupied_threshold <= 100:
        raise ValueError("occupied_threshold must be 1..100")
    # Include both occupied-cell and target-cell half diagonals. Any point
    # inside a returned free cell then retains at least the requested radius.
    inflation = radius + sqrt(2) * grid.resolution
    reach = ceil(inflation / grid.resolution)
    offsets = [(dx, dy) for dy in range(-reach, reach + 1)
               for dx in range(-reach, reach + 1)
               if hypot(dx, dy) * grid.resolution <= inflation]
    blocked = set()
    for row in range(grid.height):
        for col in range(grid.width):
            value = grid.cells[row * grid.width + col]
            if value < 0 or value >= occupied_threshold:
                for dx, dy in offsets:
                    x, y = col + dx, row + dy
                    if 0 <= x < grid.width and 0 <= y < grid.height:
                        blocked.add((x, y))
            # Outside the supplied grid is unknown as well.
            if min(col, row, grid.width - col - 1, grid.height - row - 1) * grid.resolution < radius:
                blocked.add((col, row))
    return blocked


def find_path(grid, start_xy, goal_xy, radius, occupied_threshold=50):
    """Return four-connected cell-center path or None when no path is observed.

    Start/goal must be exactly cell centers, so no unchecked connector segment
    is silently added. The caller must collision-check connectors/smoothing in
    the eventual trajectory planner. No diagonal corner cuts are permitted.
    """
    start, goal = grid.world_to_cell(*start_xy), grid.world_to_cell(*goal_xy)
    for point, cell in ((start_xy, start), (goal_xy, goal)):
        if hypot(*(a - b for a, b in zip(point, grid.cell_to_world(*cell)))) > 1e-8:
            raise ValueError("start/goal must be grid-cell centers; validate connectors separately")
        if not (0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height):
            return None
    blocked = blocked_cells(grid, radius, occupied_threshold)
    if start in blocked or goal in blocked:
        return None
    queue, costs, parent = [(0, start)], {start: 0}, {}
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return [grid.cell_to_world(*cell) for cell in reversed(path)]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = current[0] + dx, current[1] + dy
            if not (0 <= nxt[0] < grid.width and 0 <= nxt[1] < grid.height) or nxt in blocked:
                continue
            cost = costs[current] + 1
            if cost >= costs.get(nxt, float("inf")):
                continue
            costs[nxt], parent[nxt] = cost, current
            heuristic = abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
            heapq.heappush(queue, (cost + heuristic, nxt))
    return None
