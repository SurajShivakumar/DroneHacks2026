"""Predictive world model for a layout that is not known in advance.

MotionModel integrates yaw-rate and speed commands. OccupancyModel reads an
observed voxel map. WorldModel imagines the resulting poses and keeps only
futures that stay in observed free space. Goals are caller data. Nothing here
is a checkpoint list, a mesh, or a saved trajectory, so a new track is a new
map plus a new goal list rather than a new planner.

Unknown cells are not entered. Out-of-map space is not explored. Predicted
time is imagination, not a measured flight. This is not an arming path, an
estimator, or a clearance certificate: the caller still has to inflate for the
real swept envelope and recheck any trajectory before publishing a setpoint.
"""

from dataclasses import dataclass
import heapq
from math import atan2, ceil, cos, floor, hypot, isfinite, sin, sqrt

from dronebench.depth_mapping import DepthMap
from dronebench.grid import blocked_cells, find_path


def _wrap(angle):
    return (angle + 3.141592653589793) % (2 * 3.141592653589793) - 3.141592653589793


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float
    speed: float

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.x, self.y, self.z, self.yaw, self.speed)):
            raise ValueError("pose values must be finite")
        if self.speed < 0:
            raise ValueError("speed must be nonnegative")


@dataclass(frozen=True)
class MotionLimits:
    max_speed: float
    max_accel: float
    max_yaw_rate: float

    def __post_init__(self):
        for name in ("max_speed", "max_accel", "max_yaw_rate"):
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class Goal:
    """One caller-supplied target. The label is recorded and never interpreted."""

    label: str
    position: tuple
    tolerance: float

    def __post_init__(self):
        if not isinstance(self.label, str) or not self.label.strip():
            raise ValueError("goal label required")
        try:
            position = tuple(float(v) for v in self.position)
        except (TypeError, ValueError):
            position = ()
        if len(position) != 3 or not all(isfinite(v) for v in position):
            raise ValueError("goal position must be three finite coordinates")
        object.__setattr__(self, "position", position)
        if not isfinite(self.tolerance) or self.tolerance <= 0:
            raise ValueError("tolerance must be positive and finite")


class GoalSequence:
    """Ordered targets supplied as data. Reaching a later target does not skip."""

    def __init__(self, goals):
        goals = tuple(goals)
        if not goals or any(not isinstance(goal, Goal) for goal in goals):
            raise ValueError("at least one Goal required")
        self._goals = goals
        self._index = 0

    @property
    def current(self):
        return None if self.complete else self._goals[self._index]

    @property
    def complete(self):
        return self._index == len(self._goals)

    @property
    def accepted(self):
        return self._goals[:self._index]

    def observe(self, pose):
        if not isinstance(pose, Pose):
            raise ValueError("Pose required")
        if not self.complete:
            goal = self._goals[self._index]
            error = ((pose.x - goal.position[0]) ** 2 + (pose.y - goal.position[1]) ** 2
                     + (pose.z - goal.position[2]) ** 2)
            if sqrt(error) <= goal.tolerance:
                self._index += 1
        return "COMPLETE" if self.complete else self._goals[self._index].label


@dataclass(frozen=True)
class ImaginedPlan:
    kind: str
    band: tuple
    path: tuple
    poses: tuple
    predicted_seconds: float

    def __post_init__(self):
        if self.kind not in ("goal", "frontier"):
            raise ValueError("kind must be goal or frontier")
        if not self.path or not self.poses:
            raise ValueError("imagined plan requires a path and poses")
        if not isfinite(self.predicted_seconds) or self.predicted_seconds < 0:
            raise ValueError("predicted time must be finite and nonnegative")


class MotionModel:
    """Kinematic transition. Not a PX4 model and not a learned dynamics net."""

    def __init__(self, limits):
        if not isinstance(limits, MotionLimits):
            raise ValueError("MotionLimits required")
        self.limits = limits

    def predict(self, pose, yaw_rate, speed_cmd, dt):
        if not isinstance(pose, Pose):
            raise ValueError("Pose required")
        if not isfinite(dt) or dt <= 0 or not isfinite(yaw_rate) or not isfinite(speed_cmd):
            raise ValueError("finite positive dt and finite commands required")
        limits = self.limits
        yaw_rate = max(-limits.max_yaw_rate, min(limits.max_yaw_rate, yaw_rate))
        speed_cmd = max(0.0, min(limits.max_speed, speed_cmd))
        delta = max(-limits.max_accel * dt, min(limits.max_accel * dt, speed_cmd - pose.speed))
        speed = pose.speed + delta
        yaw = _wrap(pose.yaw + yaw_rate * dt)
        return Pose(pose.x + speed * cos(yaw) * dt, pose.y + speed * sin(yaw) * dt,
                    pose.z, yaw, speed)


class OccupancyModel:
    """Queries the live observed voxel map. Updates to that map are visible."""

    def __init__(self, mapping):
        if not isinstance(mapping, DepthMap):
            raise ValueError("DepthMap required")
        self.mapping = mapping

    def occupancy(self, x, y, z):
        point = (x, y, z)
        if not all(isfinite(v) for v in point):
            raise ValueError("query must be finite")
        index = tuple(int(floor(v)) for v in
                      ((x - self.mapping.origin[0]) / self.mapping.resolution,
                       (y - self.mapping.origin[1]) / self.mapping.resolution,
                       (z - self.mapping.origin[2]) / self.mapping.resolution))
        if not all(0 <= v < n for v, n in zip(index, self.mapping.shape)):
            return None
        return int(self.mapping.cells[index])

    def clear(self, pose, radius, half_height):
        """True only when the vertical body cylinder lies in observed free cells."""
        if not isinstance(pose, Pose):
            raise ValueError("Pose required")
        if not isfinite(radius) or radius < 0 or not isfinite(half_height) or half_height <= 0:
            raise ValueError("radius must be nonnegative and half_height positive")
        mapping = self.mapping
        resolution = mapping.resolution
        origin = mapping.origin
        shape = mapping.shape
        top = [origin[axis] + shape[axis] * resolution for axis in range(3)]
        z0, z1 = pose.z - half_height, pose.z + half_height
        if (pose.x - radius < origin[0] or pose.y - radius < origin[1] or z0 < origin[2]
                or pose.x + radius >= top[0] or pose.y + radius >= top[1] or z1 > top[2]):
            return False
        ix0 = int(floor((pose.x - radius - origin[0]) / resolution))
        ix1 = int(floor((pose.x + radius - origin[0]) / resolution))
        iy0 = int(floor((pose.y - radius - origin[1]) / resolution))
        iy1 = int(floor((pose.y + radius - origin[1]) / resolution))
        iz0 = int(floor((z0 - origin[2]) / resolution))
        iz1 = int(floor((z1 - origin[2]) / resolution - 1e-12))
        radius_sq = radius * radius + 1e-12
        for ix in range(ix0, ix1 + 1):
            x0 = origin[0] + ix * resolution
            nearest_x = min(max(pose.x, x0), x0 + resolution)
            for iy in range(iy0, iy1 + 1):
                y0 = origin[1] + iy * resolution
                nearest_y = min(max(pose.y, y0), y0 + resolution)
                if (nearest_x - pose.x) ** 2 + (nearest_y - pose.y) ** 2 > radius_sq:
                    continue
                for iz in range(iz0, iz1 + 1):
                    cell_z = origin[2] + iz * resolution
                    if cell_z < z1 - 1e-9 and cell_z + resolution > z0 + 1e-9:
                        if int(mapping.cells[ix, iy, iz]) != 0:
                            return False
        return True


class WorldModel:
    """Imagine motion inside the observed map and plan toward caller goals.

    A goal path is used when one altitude band shows a free route. Otherwise
    the plan stops at the reachable frontier that most reduces remaining
    distance, and the same object replans after the map changes. Altitude is
    chosen from the bands that are actually free, not from a named opening.
    """

    def __init__(self, mapping, motion, *, dt):
        self.occupancy = OccupancyModel(mapping) if isinstance(mapping, DepthMap) else mapping
        if not isinstance(self.occupancy, OccupancyModel):
            raise ValueError("DepthMap or OccupancyModel required")
        if not isinstance(motion, MotionModel):
            raise ValueError("MotionModel required")
        if not isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        self.motion = motion
        self.dt = float(dt)

    def imagine(self, pose, controls):
        """Open-loop rollout. Each control is (yaw_rate, speed_cmd) held for one dt."""
        controls = tuple(controls)
        if not controls:
            raise ValueError("at least one control required")
        poses = [pose]
        for yaw_rate, speed_cmd in controls:
            pose = self.motion.predict(pose, yaw_rate, speed_cmd, self.dt)
            poses.append(pose)
        return tuple(poses)

    def plan(self, pose, goal_xy, bands, *, radius, half_height):
        if not isinstance(pose, Pose):
            raise ValueError("Pose required")
        if len(goal_xy) != 2 or not all(isfinite(v) for v in goal_xy):
            raise ValueError("finite goal XY required")
        bands = tuple((float(z0), float(z1)) for z0, z1 in bands)
        if not bands:
            raise ValueError("at least one altitude band required")
        self._validate_body(bands, radius, half_height)
        goal_plans, frontier_plans = [], []
        for band in bands:
            planned = self._plan_band(pose, goal_xy, band, radius, half_height)
            if planned is None:
                continue
            (goal_plans if planned.kind == "goal" else frontier_plans).append(planned)
        if goal_plans:
            return min(goal_plans, key=lambda item: _path_length(item.path))
        if frontier_plans:
            return min(frontier_plans, key=lambda item: _path_length(item.path) + hypot(
                item.path[-1][0] - goal_xy[0], item.path[-1][1] - goal_xy[1]))
        return None

    def plan_current(self, pose, goals, bands, *, radius, half_height):
        if not isinstance(goals, GoalSequence):
            raise ValueError("GoalSequence required")
        goal = goals.current
        if goal is None:
            return None
        return self.plan(pose, goal.position[:2], bands, radius=radius, half_height=half_height)

    def _validate_body(self, bands, radius, half_height):
        if not isfinite(radius) or radius < 0 or not isfinite(half_height) or half_height <= 0:
            raise ValueError("radius must be nonnegative and half_height positive")
        for z0, z1 in bands:
            if not (isfinite(z0) and isfinite(z1) and z0 < z1):
                raise ValueError("altitude band must be a finite increasing pair")
            mid = (z0 + z1) / 2
            if mid - half_height < z0 or mid + half_height > z1:
                raise ValueError("vehicle height does not fit in altitude band")

    def _plan_band(self, pose, goal_xy, band, radius, half_height):
        grid = self.occupancy.mapping.occupancy_slice(*band)
        start = _snap(grid, (pose.x, pose.y))
        goal = _snap(grid, goal_xy)
        if start is None or goal is None:
            return None
        path = find_path(grid, start[1], goal[1], radius)
        kind = "goal"
        if path is None:
            path = _frontier_path(grid, start[0], goal_xy, radius)
            kind = "frontier"
        if not path:
            return None
        z = (band[0] + band[1]) / 2
        imagined = self._follow(path, Pose(path[0][0], path[0][1], z, pose.yaw, 0.0), grid.resolution)
        if imagined is None or any(not self.occupancy.clear(item, radius, half_height)
                                   for item in imagined):
            return None
        return ImaginedPlan(kind, band, tuple(path), imagined, (len(imagined) - 1) * self.dt)

    def _follow(self, path, pose, resolution):
        # Stay inside the cell being tracked. One step also cannot jump a cell.
        arrive = 0.25 * resolution
        if hypot(pose.x - path[-1][0], pose.y - path[-1][1]) <= arrive and pose.speed == 0:
            return (pose,)
        index = 0
        poses = [pose]
        limits = self.motion.limits
        speed_cap = min(limits.max_speed, arrive / self.dt)
        for _ in range(8000):
            goal_dist = hypot(pose.x - path[-1][0], pose.y - path[-1][1])
            if goal_dist <= arrive and pose.speed <= limits.max_accel * self.dt:
                return tuple(poses)
            while index < len(path) - 1 and hypot(
                    pose.x - path[index][0], pose.y - path[index][1]) <= arrive:
                index += 1
            target = path[index]
            error = _wrap(atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)
            yaw_rate = max(-limits.max_yaw_rate, min(limits.max_yaw_rate, error / self.dt))
            if goal_dist <= arrive or abs(error) > 0.2:
                speed_cmd = 0.0
            else:
                stopping = pose.speed ** 2 / (2 * limits.max_accel)
                speed_cmd = 0.0 if stopping >= max(0.0, goal_dist - arrive) else speed_cap
            pose = self.motion.predict(pose, yaw_rate, speed_cmd, self.dt)
            poses.append(pose)
        return None


def _snap(grid, xy):
    cell = grid.world_to_cell(*xy)
    if not (0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height):
        return None
    return cell, grid.cell_to_world(*cell)


def _path_length(path):
    return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def _dijkstra(grid, start, blocked):
    costs, parent = {start: 0}, {}
    queue = [(0, start)]
    while queue:
        cost, current = heapq.heappop(queue)
        if cost != costs.get(current):
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = current[0] + dx, current[1] + dy
            if not (0 <= nxt[0] < grid.width and 0 <= nxt[1] < grid.height) or nxt in blocked:
                continue
            nxt_cost = cost + 1
            if nxt_cost < costs.get(nxt, float("inf")):
                costs[nxt] = nxt_cost
                parent[nxt] = current
                heapq.heappush(queue, (nxt_cost, nxt))
    return costs, parent


def _frontier_path(grid, start, goal_xy, radius):
    blocked = blocked_cells(grid, radius)
    if start in blocked:
        return None
    costs, parent = _dijkstra(grid, start, blocked)
    inflation = radius + sqrt(2) * grid.resolution
    unknown_limit = inflation + grid.resolution
    reach = ceil(unknown_limit / grid.resolution)
    best, best_score = None, float("inf")
    for cell, cost in costs.items():
        if not _faces_unknown(grid, cell, reach, unknown_limit, inflation):
            continue
        center = grid.cell_to_world(*cell)
        score = cost * grid.resolution + hypot(center[0] - goal_xy[0], center[1] - goal_xy[1])
        if score < best_score - 1e-9 or (abs(score - best_score) <= 1e-9 and (best is None or cell < best)):
            best, best_score = cell, score
    if best is None:
        return None
    cells = [best]
    while cells[-1] != start:
        cells.append(parent[cells[-1]])
    return [grid.cell_to_world(*cell) for cell in reversed(cells)]


def _faces_unknown(grid, cell, reach, unknown_limit, occupied_limit, occupied_threshold=50):
    near_unknown = near_occupied = False
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            dist = hypot(dx, dy) * grid.resolution
            if dist > unknown_limit:
                continue
            col, row = cell[0] + dx, cell[1] + dy
            if not (0 <= col < grid.width and 0 <= row < grid.height):
                continue
            raw = grid.cells[row * grid.width + col]
            if raw < 0:
                near_unknown = True
            elif raw >= occupied_threshold and dist <= occupied_limit:
                near_occupied = True
    return near_unknown and not near_occupied
