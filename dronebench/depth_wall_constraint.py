"""Offline scalar wall-normal observations; never a complete XY position fix.

Calibrated base-frame XY points and explicit published finite wall segments are
the only geometry inputs. A trusted local seed supplies association and yaw.
The caller owns calibration, timestamps, yaw quality and map/frame validity.
Wrong periodic stereo phases or a wrong map association can remain consistent:
acceptance here cannot certify depth, global localization or flight readiness.
"""

from dataclasses import dataclass
from math import isfinite

import numpy as np


@dataclass(frozen=True)
class WallConstraintConfig:
    association_distance: float = .15
    association_ambiguity_margin: float = .03
    distinct_plane_distance: float = .04
    normal_cluster_angle: float = .001
    sample_spacing: float = .08
    min_inliers: int = 8
    min_inlier_ratio: float = .7
    residual_gate: float = .03
    max_rms_residual: float = .025
    min_spatial_spread: float = .35
    max_innovation: float = .12
    uncertainty_floor: float = .03
    yaw_std_rad: float = .02
    secondary_mode_fraction: float = .2
    min_secondary_mode: int = 4

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ("min_inliers", "min_secondary_mode"):
                if type(value) is not int or value <= 0:
                    raise ValueError(f"{name} must be a positive integer")
            elif not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.min_inlier_ratio > 1 or self.secondary_mode_fraction > .5:
            raise ValueError("inlier ratio must be <=1 and secondary_mode_fraction <=.5")
        if self.normal_cluster_angle > .01:
            raise ValueError("normal clusters must remain within .01 radians")


@dataclass(frozen=True)
class WallConstraint:
    accepted: bool
    reason: str
    normal: tuple[float, float]
    projection: float | None
    variance: float | None
    candidate_projection: float | None
    innovation: float | None
    candidates: int
    inliers: int
    inlier_ratio: float
    spatial_spread: float
    rms_residual: float | None
    segment_indices: tuple[int, ...]


@dataclass(frozen=True)
class WallConstraintResult:
    constraints: tuple[WallConstraint, ...]
    reason: str
    finite_points: int
    invalid_points: int
    sampled_points: int
    associated_points: int
    ambiguous_points: int


def extract_wall_constraints(points_base_xy, wall_segments, seed_pose, *, config=None):
    """Estimate independent scalar measurements ``normal dot position``.

    Rejected factors expose only a diagnostic candidate, not a measurement or
    variance. Each accepted factor has rank one: all translations orthogonal to
    its normal remain unobserved. Factors share seed yaw/calibration errors and
    must not be treated as independent repeated-frame observations.

    Normals have canonical sign. Spatial sampling limits pixel-density support;
    variance retains full residual scatter, a 3cm default floor and a full
    range*yaw uncertainty contribution, without dividing by point count. Small
    angular normal clusters use the seed's tangential projection explicitly,
    with their angular approximation included in the uncertainty bound.
    """
    cfg = WallConstraintConfig() if config is None else config
    if not isinstance(cfg, WallConstraintConfig):
        raise ValueError("config must be WallConstraintConfig")
    points = np.asarray(points_base_xy, float)
    walls = np.asarray(wall_segments, float)
    seed = np.asarray(seed_pose, float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points_base_xy must have shape Nx2")
    if walls.ndim != 3 or walls.shape[1:] != (2, 2) or len(walls) == 0 or not np.isfinite(walls).all():
        raise ValueError("finite nonempty wall_segments Mx2x2 required")
    if seed.shape != (3,) or not np.isfinite(seed).all():
        raise ValueError("finite seed_pose x,y,yaw required")
    with np.errstate(over="ignore", invalid="ignore"):
        delta = walls[:, 1] - walls[:, 0]
        length = np.linalg.norm(delta, axis=1)
    if not np.isfinite(length).all() or np.any(length <= 1e-9):
        raise ValueError("finite nonzero wall lengths required")
    tangent = delta / length[:, None]
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    flip = (normals[:, 0] < -1e-12) | ((np.abs(normals[:, 0]) <= 1e-12) & (normals[:, 1] < 0))
    normals[flip] *= -1
    groups, representatives = [], []
    for normal in normals:
        group = next((i for i, n in enumerate(representatives)
                      if np.dot(n, normal) >= np.cos(cfg.normal_cluster_angle)), None)
        if group is None:
            group = len(representatives)
            representatives.append(normal.copy())
        groups.append(group)
    groups = np.asarray(groups)
    finite = np.isfinite(points).all(axis=1)
    invalid = int((~finite).sum())
    points = points[finite]
    finite_count = len(points)
    if len(points):
        with np.errstate(over="ignore"):
            bins = np.floor(points / cfg.sample_spacing)
        if not np.isfinite(bins).all():
            raise ValueError("sensor coordinates exceed numerical range")
        _, indices = np.unique(bins, axis=0, return_index=True)
        points = points[np.sort(indices)]
    if len(points) < cfg.min_inliers:
        return WallConstraintResult((), "insufficient_points", finite_count, invalid, len(points), 0, 0)
    c, s = np.cos(seed[2]), np.sin(seed[2])
    with np.errstate(over="ignore", invalid="ignore"):
        rotated = points @ np.array([[c, s], [-s, c]])
        world = rotated + seed[:2]
        offset = world[:, None] - walls[None, :, 0]
        along = np.einsum("nmi,mi->nm", offset, tangent)
        signed = np.einsum("nmi,mi->nm", offset, normals)
    if not all(np.isfinite(a).all() for a in (rotated, world, along, signed)):
        raise ValueError("transformed coordinates exceed numerical range")
    distance = np.abs(signed)
    eligible = (along >= 0) & (along <= length) & (distance <= cfg.association_distance)
    costs = np.where(eligible, distance, np.inf)
    association = np.argmin(costs, axis=1)
    rows = np.arange(len(points))
    associated = np.isfinite(costs[rows, association])
    ambiguous = np.zeros(len(points), bool)
    for j in range(len(walls)):
        chosen = association
        parallel = groups[j] == groups[chosen]
        # Same/coplanar segments are harmless duplicates, separate plausible
        # parallel planes are competing associations and do not establish z.
        plane_gap = np.abs(np.einsum("ni,ni->n", normals[chosen], walls[j, 0] - walls[chosen, 0]))
        ambiguous |= (associated & eligible[:, j] & parallel
                      & (plane_gap > cfg.distinct_plane_distance)
                      & (costs[:, j] <= costs[rows, chosen] + cfg.association_ambiguity_margin))
    factors = []
    for group, normal in enumerate(representatives):
        candidates = associated & (groups[association] == group)
        if not candidates.any():
            continue
        number = int(candidates.sum())
        chosen = association[candidates]
        segment_ids = tuple(int(i) for i in np.unique(chosen))
        normal_tuple = tuple(float(v) for v in normal)
        if ambiguous[candidates].any():
            factors.append(WallConstraint(False, "ambiguous_parallel_association", normal_tuple,
                                          None, None, None, None, number, 0, 0., 0., None, segment_ids))
            continue
        n_i = normals[chosen]
        q = rotated[candidates]
        seed_projection = float(normal @ seed[:2])
        # Convert almost-parallel measurements to the cluster representative
        # while holding the unobserved tangent at the seed, never estimating it.
        with np.errstate(over="ignore", invalid="ignore"):
            innovations = (np.einsum("ni,ni->n", n_i, walls[chosen, 0] - q - seed[:2])
                           / (n_i @ normal))
            values = seed_projection + innovations
        if not np.isfinite(values).all() or not isfinite(seed_projection):
            raise ValueError("wall projections exceed numerical range")
        median = float(np.median(values))
        residual = values - median
        keep = np.abs(residual) <= cfg.residual_gate
        inliers = int(keep.sum())
        ratio = inliers / number
        rms = float(np.sqrt(np.mean(residual[keep] ** 2))) if inliers else None
        axis = np.array([-normal[1], normal[0]])
        spread = float(np.ptp(np.quantile(q[keep] @ axis, [.1, .9]))) if inliers else 0.
        innovation = median - seed_projection
        reason = "accepted"
        # A compact second population is a conflicting hypothesis, even when
        # the median's first population has enough support to pass the ratio.
        other = np.sort(values[np.abs(residual) > 2 * cfg.residual_gate])
        required = max(cfg.min_secondary_mode, int(np.ceil(cfg.secondary_mode_fraction * number)))
        multimodal = any(other[i + required - 1] - other[i] <= 2 * cfg.residual_gate
                         for i in range(max(0, len(other) - required + 1)))
        if multimodal:
            reason = "multimodal_projection"
        elif inliers < cfg.min_inliers or ratio < cfg.min_inlier_ratio:
            reason = "insufficient_consistent_support"
        elif spread < cfg.min_spatial_spread:
            reason = "insufficient_spatial_spread"
        elif rms > cfg.max_rms_residual:
            reason = "residual_limit"
        elif abs(innovation) > cfg.max_innovation:
            reason = "innovation_limit"
        accepted = reason == "accepted"
        variance = None
        if accepted:
            ranges = np.linalg.norm(q[keep], axis=1)
            if not np.isfinite(ranges).all():
                raise ValueError("sensor ranges exceed numerical range")
            with np.errstate(over="ignore", invalid="ignore"):
                uncertainty = np.array([cfg.uncertainty_floor, rms,
                                        ranges.max() * (cfg.yaw_std_rad + cfg.normal_cluster_angle)])
                variance = float(uncertainty @ uncertainty)
            if not isfinite(variance):
                accepted, reason, variance = False, "uncertainty_overflow", None
        factors.append(WallConstraint(accepted, reason, normal_tuple,
                                      median if accepted else None, variance, median, innovation,
                                      number, inliers, ratio, spread, rms, segment_ids))
    reason = "accepted_constraints" if any(f.accepted for f in factors) else "no_accepted_constraints"
    return WallConstraintResult(tuple(factors), reason, finite_count, invalid, len(points),
                                int(associated.sum()), int(ambiguous.sum()))
