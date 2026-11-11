"""Height slices of caller-supplied published course OBBs, for offline alignment.

This is a partial geometric prior, not observed occupancy or a collision map.
No file, ROS, simulator/world state, or hidden arena constants are read here.
Tube shells, boulders and other non-box geometry must be treated as unmatched
sensor returns by the alignment caller, rather than forced onto these edges.
"""

from dataclasses import dataclass
from math import isfinite

import numpy as np


@dataclass(frozen=True)
class PublishedWallMap:
    segments: np.ndarray
    labels: tuple[str, ...]
    z: float
    source_scope: str
    omitted_geometry: tuple[str, ...] = ("tube curved shell", "boulders", "floor and ceiling")


def build_published_wall_map(course, *, z, net_bounds=None):
    """Return map-frame metre segments ``Mx2x2`` at the finite height ``z``.

    ``course`` is an already parsed Course-like object: box_c is Kx3 centres,
    box_u is Kx2 horizontal unit axes, box_h is Kx3 positive half extents.
    The perpendicular horizontal axis is (-u_y, u_x), matching Course.collides.
    Optional box_names label output edges; their length must match K.

    Boxes whose closed vertical intervals contain z contribute four true OBB
    edges, without axis-aligned bounding-box approximation or safety inflation.
    Top/bottom face boundaries are included at the exact height endpoints;
    callers should avoid them when horizontal-wall observations are required.
    Internal/occluded edges are retained: this is not a visibility prediction.

    ``net_bounds=(xmin,xmax,ymin,ymax)`` explicitly contributes four boundary
    segments. Passing it asserts that the net exists at the requested height;
    the function cannot infer its height, material visibility, or sensor return.
    Omitting it adds no arena boundary. Zero/negative extents and invalid axes
    are rejected rather than silently producing degenerate wall segments.
    """
    if not isfinite(z):
        raise ValueError("z must be a finite map-frame height")
    try:
        centres = np.asarray(course.box_c, dtype=float)
        axes = np.asarray(course.box_u, dtype=float)
        half = np.asarray(course.box_h, dtype=float)
    except AttributeError as exc:
        raise ValueError("course must provide box_c, box_u, box_h") from exc
    # Course's parser emits shape (0,) when no boxes were found.
    if centres.shape == (0,) and axes.shape == (0,) and half.shape == (0,):
        centres, axes, half = centres.reshape(0, 3), axes.reshape(0, 2), half.reshape(0, 3)
    if (centres.ndim != 2 or centres.shape[1] != 3
            or axes.shape != (len(centres), 2) or half.shape != centres.shape):
        raise ValueError("box arrays must have matching Kx3 centres/extents and Kx2 axes")
    if not all(np.isfinite(array).all() for array in (centres, axes, half)):
        raise ValueError("box geometry must be finite")
    if np.any(half <= 0):
        raise ValueError("box half extents must be positive")
    if not np.allclose(np.linalg.norm(axes, axis=1), 1., atol=1e-7, rtol=0):
        raise ValueError("box_u must contain unit horizontal axes")
    names = getattr(course, "box_names", tuple(str(i) for i in range(len(centres))))
    if isinstance(names, str) or len(names) != len(centres) or any(not isinstance(n, str) for n in names):
        raise ValueError("box_names must contain one string per box")
    bounds = None
    if net_bounds is not None:
        bounds = np.asarray(net_bounds, dtype=float)
        if (bounds.shape != (4,) or not np.isfinite(bounds).all()
                or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]):
            raise ValueError("net_bounds must be finite ordered xmin,xmax,ymin,ymax")

    segments, labels = [], []
    with np.errstate(over="ignore", invalid="ignore"):
        low, high = centres[:, 2] - half[:, 2], centres[:, 2] + half[:, 2]
    if not np.isfinite(low).all() or not np.isfinite(high).all():
        raise ValueError("box heights exceed numerical range")
    for index in np.flatnonzero((low <= z) & (z <= high)):
        u = axes[index]
        v = np.array([-u[1], u[0]])
        with np.errstate(over="ignore", invalid="ignore"):
            along, across = half[index, 0] * u, half[index, 1] * v
            corners = centres[index, :2] + np.array([-along - across, along - across,
                                                       along + across, -along + across])
        if not np.isfinite(corners).all():
            raise ValueError("box corners exceed numerical range")
        for edge in range(4):
            segments.append((corners[edge], corners[(edge + 1) % 4]))
            labels.append(f"box:{index}:{names[index]}:edge:{edge}")
    if bounds is not None:
        xmin, xmax, ymin, ymax = bounds
        corners = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
        for edge, side in enumerate(("ymin", "xmax", "ymax", "xmin")):
            segments.append((corners[edge], corners[(edge + 1) % 4]))
            labels.append(f"explicit_net:{side}")
    array = np.asarray(segments, dtype=float).reshape(-1, 2, 2)
    # Check even strictly positive dimensions did not collapse through precision.
    if len(array) and np.any(np.all(array[:, 0] == array[:, 1], axis=1)):
        raise ValueError("wall segment collapsed through numerical precision")
    array.setflags(write=False)
    scope = "published_course_boxes"
    if bounds is not None:
        scope += "_and_explicit_net"
    return PublishedWallMap(array, tuple(labels), float(z), scope)
