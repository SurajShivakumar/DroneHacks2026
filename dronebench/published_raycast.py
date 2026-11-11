"""Partial, offline visibility prior from caller-supplied published geometry.

No files, simulator state, ROS, or default arena dimensions are read. Geometry
provenance is an explicit caller assertion, not independently authenticated.
Netting is modeled as four opaque rectangles; actual sensor returns can differ.
This is never a complete-world visibility or collision certificate.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RaycastHits:
    t: np.ndarray
    normals: np.ndarray
    labels: np.ndarray
    origin_inside_solid: bool
    inside_box_labels: tuple[str, ...]
    source_scope: str
    omitted_geometry: tuple[str, ...]


@dataclass(frozen=True)
class PublishedRaycast:
    centres: np.ndarray
    bases: np.ndarray
    half_extents: np.ndarray
    box_labels: tuple[str, ...]
    net_bounds: np.ndarray | None
    floor_z: float | None
    published_provenance: str
    source_scope: str
    omitted_geometry: tuple[str, ...]

    def cast(self, origin, directions):
        """Intersect ``origin + t * directions`` for strictly positive t.

        Origin is one map-frame 3-vector and directions is Nx3, finite and
        nonzero per row. Rays are deliberately NOT normalized: when directions
        are rotated optical rays with camera z=1, t is optical-axis depth in
        metres, not Euclidean range. Miss t/normals are NaN and labels are None.
        Normals are outward world-frame unit normals, including for exits from
        solid boxes. A boundary origin counts as inside the closed solid.
        Ties select the first box/face in input order deterministically.
        """
        origin = np.asarray(origin, dtype=float)
        rays = np.asarray(directions, dtype=float)
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise ValueError("origin must be a finite 3-vector")
        if (rays.ndim != 2 or rays.shape[1] != 3
                or not np.isfinite(rays).all() or np.any(np.all(rays == 0, axis=1))):
            raise ValueError("directions must be finite nonzero Nx3 vectors")
        n = len(rays)
        best = np.full(n, np.inf)
        normals = np.full((n, 3), np.nan)
        labels = np.full(n, None, dtype=object)
        inside = []

        def update(t, valid, normal, label):
            select = valid & np.isfinite(t) & (t > 0) & (t < best)
            best[select] = t[select]
            normals[select] = normal if np.ndim(normal) == 1 else normal[select]
            labels[select] = label

        for centre, basis, half, label in zip(
                self.centres, self.bases, self.half_extents, self.box_labels):
            # Columns are local u, v=(-uy, ux), z in world coordinates.
            with np.errstate(over="ignore", invalid="ignore"):
                local_origin = (origin - centre) @ basis
                local_rays = rays @ basis
            if not np.isfinite(local_origin).all() or not np.isfinite(local_rays).all():
                raise ValueError("ray transform exceeds numerical range")
            if np.all(np.abs(local_origin) <= half):
                inside.append(label)
            parallel = local_rays == 0
            outside_parallel = np.any(parallel & (np.abs(local_origin) > half), axis=1)
            near = np.full((n, 3), -np.inf)
            far = np.full((n, 3), np.inf)
            with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
                a = np.divide(-half - local_origin, local_rays,
                              out=np.zeros_like(local_rays), where=~parallel)
                b = np.divide(half - local_origin, local_rays,
                              out=np.zeros_like(local_rays), where=~parallel)
            np.copyto(near, np.minimum(a, b), where=~parallel)
            np.copyto(far, np.maximum(a, b), where=~parallel)
            entry_axis = np.argmax(near, axis=1)
            exit_axis = np.argmin(far, axis=1)
            entry, leave = np.max(near, axis=1), np.min(far, axis=1)
            entering = entry > 0
            t = np.where(entering, entry, leave)
            face_axis = np.where(entering, entry_axis, exit_axis)
            component = local_rays[np.arange(n), face_axis]
            sign = np.where(entering, -np.sign(component), np.sign(component))
            normal = basis[:, face_axis].T * sign[:, None]
            update(t, (~outside_parallel) & (entry <= leave), normal, label)

        if self.net_bounds is not None:
            bounds = self.net_bounds.reshape(3, 2)
            # No implicit roof or floor: net is four finite vertical rectangles.
            for axis, side in ((0, 0), (0, 1), (1, 0), (1, 1)):
                with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
                    t = (bounds[axis, side] - origin[axis]) / rays[:, axis]
                    points = origin + t[:, None] * rays
                valid = rays[:, axis] != 0
                for other in range(3):
                    if other != axis:
                        valid &= ((points[:, other] >= bounds[other, 0])
                                  & (points[:, other] <= bounds[other, 1]))
                normal = np.zeros(3)
                normal[axis] = -1 if side == 0 else 1
                update(t, valid, normal, f"explicit_net:{'xy'[axis]}{'min' if side == 0 else 'max'}")
        if self.floor_z is not None:
            with np.errstate(divide="ignore", over="ignore", invalid="ignore"):
                t = (self.floor_z - origin[2]) / rays[:, 2]
            update(t, rays[:, 2] != 0, np.array([0., 0., 1.]), "explicit_floor")
        best[~np.isfinite(best)] = np.nan
        for array in (best, normals, labels):
            array.setflags(write=False)
        return RaycastHits(best, normals, labels, bool(inside), tuple(inside),
                           self.source_scope, self.omitted_geometry)


def build_published_raycast(course, *, published_provenance, net_bounds=None, floor_z=None):
    """Copy Course box_c/Kx3, box_u/Kx2, box_h/Kx3 into an offline prior.

    ``published_provenance`` must identify the public source the caller asserts
    supplied these arrays. No Course defaults (half_size/net_height) are used.
    Optional net_bounds=(xmin,xmax,ymin,ymax,zmin,zmax) must explicitly describe
    four finite vertical net faces. Optional floor_z adds an infinite plane.
    Box extents are physical, with no inflation; Course window decomposition
    and finite wall heights are preserved. No calibration is accepted/inferred.
    """
    if not isinstance(published_provenance, str) or not published_provenance.strip():
        raise ValueError("published_provenance must explicitly identify the published source")
    try:
        centres = np.array(course.box_c, dtype=float, copy=True)
        axes = np.array(course.box_u, dtype=float, copy=True)
        half = np.array(course.box_h, dtype=float, copy=True)
    except AttributeError as exc:
        raise ValueError("course must supply box_c, box_u, box_h") from exc
    if centres.shape == axes.shape == half.shape == (0,):
        centres, axes, half = centres.reshape(0, 3), axes.reshape(0, 2), half.reshape(0, 3)
    if (centres.ndim != 2 or centres.shape[1] != 3
            or half.shape != centres.shape or axes.shape != (len(centres), 2)):
        raise ValueError("expected matching Kx3 centres/extents and Kx2 axes")
    if not all(np.isfinite(a).all() for a in (centres, axes, half)) or np.any(half <= 0):
        raise ValueError("box geometry must be finite with positive half extents")
    if not np.allclose(np.linalg.norm(axes, axis=1), 1., atol=1e-7, rtol=0):
        raise ValueError("box_u must contain unit horizontal axes")
    names = getattr(course, "box_names", tuple(str(i) for i in range(len(centres))))
    if isinstance(names, str) or len(names) != len(centres) or any(not isinstance(s, str) for s in names):
        raise ValueError("box_names must contain one string per box")
    bases = np.zeros((len(centres), 3, 3))
    bases[:, :2, 0] = axes
    bases[:, :2, 1] = np.column_stack((-axes[:, 1], axes[:, 0]))
    bases[:, 2, 2] = 1
    with np.errstate(over="ignore", invalid="ignore"):
        world_extent = np.einsum("kij,kj->ki", np.abs(bases), half)
        low, high = centres - world_extent, centres + world_extent
    if (not np.isfinite(low).all() or not np.isfinite(high).all()
            or np.any(low >= high)):
        raise ValueError("box bounds overflow or collapse through numerical precision")
    bounds = None
    if net_bounds is not None:
        bounds = np.array(net_bounds, dtype=float, copy=True)
        if (bounds.shape != (6,) or not np.isfinite(bounds).all()
                or np.any(bounds[::2] >= bounds[1::2])):
            raise ValueError("net_bounds must be finite ordered xmin,xmax,ymin,ymax,zmin,zmax")
    if floor_z is not None:
        if np.ndim(floor_z) != 0 or not np.isfinite(float(floor_z)):
            raise ValueError("floor_z must be a finite scalar")
        floor_z = float(floor_z)
    for array in (centres, bases, half, bounds):
        if array is not None:
            array.setflags(write=False)
    scope = "published_course_boxes" + ("_and_explicit_net" if bounds is not None else "")
    scope += "_and_explicit_floor" if floor_z is not None else ""
    omissions = ("boulder cylinders", "tube curved shell", "other non-box geometry", "ceiling")
    if bounds is None:
        omissions += ("net boundary",)
    if floor_z is None:
        omissions += ("floor",)
    return PublishedRaycast(centres, bases, half,
                            tuple(f"box:{i}:{name}" for i, name in enumerate(names)),
                            bounds, floor_z, published_provenance, scope, omissions)


def raycast_published_course(course, origin, directions, *, published_provenance,
                             net_bounds=None, floor_z=None):
    """Convenience wrapper; build once and use .cast for repeated calls."""
    return build_published_raycast(
        course, published_provenance=published_provenance,
        net_bounds=net_bounds, floor_z=floor_z).cast(origin, directions)
