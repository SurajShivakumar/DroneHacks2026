"""Observed voxel occupancy from rectified optical-axis depth in metres.

Inputs are calibrated depth and a time-aligned camera-optical -> map transform
(optical X right, Y down, Z forward). No pose estimation or ROS I/O occurs here.
Only measured finite returns carve free centre rays; invalid/no-return pixels
remain unknown. Occupied voxels are sticky until an explicit map replacement.
Sparse rays do not prove the space between pixels free. A planner must inflate
unknown and occupied cells for vehicle size, calibration and pose uncertainty.
"""

from dataclasses import dataclass
from math import floor, isfinite

import numpy as np

from dronebench.grid import OccupancySlice


@dataclass(frozen=True)
class Pinhole:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if any(type(v) is not int or v <= 0 for v in (self.width, self.height)):
            raise ValueError("positive integer image dimensions required")
        if not all(isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)) or min(self.fx, self.fy) <= 0:
            raise ValueError("finite pinhole calibration with positive focal lengths required")


class DepthMap:
    """Axis-aligned fixed metric map: unknown=-1, free=0, occupied=100.

    ``shape`` is (nx, ny, nz); cells use that same axis order. ``origin`` is
    the minimum corner in the caller's map frame. Integrate only observations
    whose pose and timestamps have been checked by the caller. This component
    does not establish freshness, localization quality or collision safety.
    """

    def __init__(self, shape, resolution, origin=(0., 0., 0.)):
        if len(shape) != 3 or any(type(v) is not int or v <= 0 for v in shape):
            raise ValueError("shape must contain three positive integers")
        if not isfinite(resolution) or resolution <= 0:
            raise ValueError("resolution must be positive and finite")
        if len(origin) != 3 or not all(isfinite(v) for v in origin):
            raise ValueError("origin must contain three finite coordinates")
        self.shape = tuple(shape)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.cells = np.full(shape, -1, dtype=np.int8)

    def _index(self, point):
        return tuple(int(floor(v)) for v in (point - self.origin) / self.resolution)

    def _inside(self, index):
        return all(0 <= v < n for v, n in zip(index, self.shape))

    def _ray(self, start, end):
        """Clip a segment to map bounds and enumerate crossed voxel interiors."""
        delta = end - start
        lo, hi = 0., 1.
        upper = self.origin + np.asarray(self.shape) * self.resolution
        for axis in range(3):
            if abs(delta[axis]) < 1e-12:
                if not self.origin[axis] <= start[axis] < upper[axis]:
                    return
            else:
                a, b = sorted(((self.origin[axis] - start[axis]) / delta[axis],
                               (upper[axis] - start[axis]) / delta[axis]))
                lo, hi = max(lo, a), min(hi, b)
        if hi <= lo:
            return
        cuts = [lo, hi]
        for axis in range(3):
            if abs(delta[axis]) < 1e-12:
                continue
            a = (start[axis] + delta[axis] * lo - self.origin[axis]) / self.resolution
            b = (start[axis] + delta[axis] * hi - self.origin[axis]) / self.resolution
            first = max(1, floor(min(a, b)) + 1)
            last = min(self.shape[axis], int(np.ceil(max(a, b))))
            for boundary in range(first, last):
                t = (self.origin[axis] + boundary * self.resolution - start[axis]) / delta[axis]
                if lo < t < hi:
                    cuts.append(float(t))
        cuts = sorted(set(cuts))
        for a, b in zip(cuts, cuts[1:]):
            index = self._index(start + delta * ((a + b) / 2))
            if self._inside(index):
                yield index

    def integrate(self, depth_m, camera, camera_to_map, *, min_depth=.1, max_depth=8., stride=1):
        """Fuse a depth image; return counts of valid and invalid sampled pixels.

        Depth is Z, not Euclidean range. camera_to_map is a proper 4x4 rigid
        matrix, e.g. from calibrated TF and estimated pose at the image stamp.
        Convert integer millimetre images before calling. Out-of-range values,
        including infinity, never mark free space. An endpoint outside this map
        can still clear the observed portion of its ray inside the map.
        """
        depth = np.asarray(depth_m)
        if depth.shape != (camera.height, camera.width) or depth.dtype.kind != "f":
            raise ValueError("depth must be a floating metre image matching calibration")
        transform = np.asarray(camera_to_map, dtype=float)
        if transform.shape != (4, 4) or not np.isfinite(transform).all():
            raise ValueError("finite 4x4 transform required")
        rotation = transform[:3, :3]
        if (not np.allclose(transform[3], (0, 0, 0, 1), atol=1e-8, rtol=0)
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0)):
            raise ValueError("transform must be proper rigid camera-optical -> map")
        if not (isfinite(min_depth) and isfinite(max_depth) and 0 < min_depth < max_depth):
            raise ValueError("finite ordered positive depth limits required")
        if type(stride) is not int or stride <= 0:
            raise ValueError("stride must be a positive integer")
        start = transform[:3, 3]
        occupied, free = set(), set()
        valid = invalid = 0
        for v in range(0, camera.height, stride):
            for u in range(0, camera.width, stride):
                z = float(depth[v, u])
                if not isfinite(z) or not min_depth <= z <= max_depth:
                    invalid += 1
                    continue
                valid += 1
                optical = np.array(((u - camera.cx) * z / camera.fx,
                                    (v - camera.cy) * z / camera.fy, z))
                end = rotation @ optical + start
                endpoint = self._index(end)
                if self._inside(endpoint):
                    occupied.add(endpoint)
                free.update(index for index in self._ray(start, end) if index != endpoint)
        for index in free - occupied:
            if self.cells[index] != 100:
                self.cells[index] = 0
        for index in occupied:
            self.cells[index] = 100
        return {"valid_returns": valid, "invalid_pixels": invalid}

    def occupancy_slice(self, z_min, z_max):
        """Collapse a full height envelope; all intersected voxels must be free.

        Bounds are map-frame metres, with z_max exclusive. Out-of-map envelopes
        are rejected; no unobserved vertical space is silently discarded.
        """
        top = self.origin[2] + self.shape[2] * self.resolution
        if not (isfinite(z_min) and isfinite(z_max) and self.origin[2] <= z_min < z_max <= top):
            raise ValueError("height envelope must lie inside map")
        low = floor((z_min - self.origin[2]) / self.resolution)
        high = int(np.ceil((z_max - self.origin[2]) / self.resolution))
        slab = self.cells[:, :, low:high]
        projected = np.where(np.any(slab == 100, axis=2), 100,
                             np.where(np.all(slab == 0, axis=2), 0, -1))
        return OccupancySlice(self.shape[0], self.shape[1], self.resolution,
                              tuple(int(v) for v in projected.T.flat),
                              float(self.origin[0]), float(self.origin[1]))
