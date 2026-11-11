"""Offline, local SE(2) alignment to measured/published static wall geometry.

Inputs are calibrated points already transformed into base-frame XY (metres),
not optical XY or raw depth pixels. Pose maps base XY into the map using a
counterclockwise yaw in radians. The caller owns TF, timestamps, wall selection,
tilt compensation and seed validity. This module supplies no position fix,
freshness check, ROS publication or authority to fly. Repeated map geometry can
produce a locally consistent wrong match: acceptance is a local diagnostic only.
"""

from dataclasses import dataclass
from math import isfinite, pi

import numpy as np


@dataclass(frozen=True)
class AlignmentConfig:
    max_iterations: int = 40
    min_inliers: int = 12
    min_inlier_ratio: float = .5
    correspondence_distance: float = .35
    huber_distance: float = .05
    sample_spacing: float = .15
    max_step_translation: float = .10
    max_step_yaw: float = 5 * pi / 180
    max_translation_correction: float = .40
    max_yaw_correction: float = 15 * pi / 180
    translation_tolerance: float = 1e-5
    yaw_tolerance: float = 1e-5
    min_information_ratio: float = .001
    max_rms_residual: float = .08
    noise_floor: float = .03
    translation_uncertainty_floor: float = .03
    yaw_uncertainty_floor: float = 2 * pi / 180
    max_effective_samples: int = 20

    def __post_init__(self):
        for name in ("max_iterations", "min_inliers", "max_effective_samples"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name, value in vars(self).items():
            if name in ("max_iterations", "min_inliers", "max_effective_samples"):
                continue
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.min_inlier_ratio > 1 or self.min_information_ratio >= 1:
            raise ValueError("inlier ratio must be <= 1 and information ratio < 1")


@dataclass(frozen=True)
class AlignmentResult:
    accepted: bool
    reason: str
    pose: tuple[float, float, float]
    candidate_pose: tuple[float, float, float]
    iterations: int
    finite_points: int
    invalid_points: int
    sampled_points: int
    inliers: int
    inlier_ratio: float
    rms_residual: float | None
    covariance: np.ndarray | None


def _angle(value):
    return (value + pi) % (2 * pi) - pi


def align_points_to_segments(points_base_xy, wall_segments, seed_pose, *, config=None):
    """Robust point-to-segment ICP; rejection retains the seed in ``pose``.

    Malformed arrays, nonfinite seeds/maps and zero-length segments raise
    ValueError. Nonfinite sensor rows are discarded and counted. Spatial
    subsampling prevents repeated pixels from increasing support; min_inliers
    and inlier_ratio refer to these samples. Finite segment endpoints only gate
    association: they never create a tangential constraint on a single wall.

    Covariance is a heuristic local uncertainty in map x/y/yaw, with density
    reduction, an effective-sample cap, noise and systematic uncertainty floors.
    It is not a calibrated estimator covariance and excludes map/calibration
    bias and alternate associations. Rejected candidates have no covariance.
    """
    cfg = config if config is not None else AlignmentConfig()
    if not isinstance(cfg, AlignmentConfig):
        raise ValueError("config must be AlignmentConfig")
    points = np.asarray(points_base_xy, dtype=float)
    walls = np.asarray(wall_segments, dtype=float)
    seed = np.asarray(seed_pose, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points_base_xy must have shape Nx2")
    if walls.ndim != 3 or walls.shape[1:] != (2, 2) or len(walls) == 0:
        raise ValueError("wall_segments must have nonempty shape Mx2x2")
    if seed.shape != (3,) or not np.isfinite(seed).all():
        raise ValueError("seed_pose must contain finite x, y, yaw")
    if not np.isfinite(walls).all():
        raise ValueError("wall_segments must be finite")
    delta = walls[:, 1] - walls[:, 0]
    lengths = np.linalg.norm(delta, axis=1)
    if not np.isfinite(lengths).all() or np.any(lengths <= 1e-9):
        raise ValueError("wall segments must have finite nonzero length")
    tangent = delta / lengths[:, None]
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    finite = np.isfinite(points).all(axis=1)
    invalid = int((~finite).sum())
    points = points[finite]
    finite_count = len(points)
    # np.unique on floating bins avoids integer overflow for extreme inputs.
    if len(points):
        with np.errstate(over="ignore"):
            bins = np.floor(points / cfg.sample_spacing)
        if not np.isfinite(bins).all():
            raise ValueError("sensor coordinates exceed numerical range")
        _, indices = np.unique(bins, axis=0, return_index=True)
        points = points[np.sort(indices)]
    pose = seed.copy()
    iteration, count, ratio, rms = 0, 0, 0., None

    def result(accepted, reason, covariance=None):
        return AlignmentResult(accepted, reason, tuple(pose if accepted else seed),
                               tuple(pose), iteration, finite_count, invalid,
                               len(points), count, ratio, rms, covariance)

    if len(points) < cfg.min_inliers:
        return result(False, "insufficient_points")

    def correspondences(current):
        c, s = np.cos(current[2]), np.sin(current[2])
        rotated = points @ np.array([[c, s], [-s, c]])
        world = rotated + current[:2]
        offset = world[:, None, :] - walls[None, :, 0, :]
        along = np.einsum("nmi,mi->nm", offset, tangent)
        closest = walls[None, :, 0, :] + np.clip(along, 0, lengths)[:, :, None] * tangent
        distance = np.linalg.norm(world[:, None, :] - closest, axis=2)
        association = np.argmin(distance, axis=1)
        rows = np.arange(len(points))
        valid = ((distance[rows, association] <= cfg.correspondence_distance)
                 & (along[rows, association] >= 0)
                 & (along[rows, association] <= lengths[association]))
        n = normals[association[valid]]
        residual = np.einsum("ni,ni->n", n,
                             world[valid] - walls[association[valid], 0])
        q = rotated[valid]
        jacobian = np.column_stack((n, -n[:, 0] * q[:, 1] + n[:, 1] * q[:, 0]))
        weights = np.minimum(1., cfg.huber_distance / np.maximum(np.abs(residual), 1e-12))
        return residual, jacobian, weights

    converged = False
    for iteration in range(1, cfg.max_iterations + 1):
        residual, jacobian, weights = correspondences(pose)
        count = len(residual)
        ratio = count / len(points)
        rms = float(np.sqrt(np.mean(residual ** 2))) if count else None
        if count < cfg.min_inliers or ratio < cfg.min_inlier_ratio:
            return result(False, "insufficient_inliers")
        hessian = jacobian.T @ (weights[:, None] * jacobian)
        # Normalize columns so observability does not depend on metres vs radians.
        scales = np.sqrt(np.maximum(np.diag(hessian), 1e-15))
        normalized = hessian / scales[:, None] / scales[None, :]
        eigenvalues = np.linalg.eigvalsh(normalized)
        if eigenvalues[0] <= cfg.min_information_ratio * eigenvalues[-1]:
            return result(False, "unobservable_geometry")
        step = np.linalg.solve(hessian, -jacobian.T @ (weights * residual))
        if not np.isfinite(step).all():
            return result(False, "numerical_failure")
        scale = min(1., cfg.max_step_translation / max(np.linalg.norm(step[:2]), 1e-15),
                    cfg.max_step_yaw / max(abs(step[2]), 1e-15))
        step *= scale
        candidate = pose + step
        candidate[2] = seed[2] + _angle(candidate[2] - seed[2])
        if (np.linalg.norm(candidate[:2] - seed[:2]) > cfg.max_translation_correction
                or abs(_angle(candidate[2] - seed[2])) > cfg.max_yaw_correction):
            return result(False, "correction_limit")
        if (np.linalg.norm(step[:2]) <= cfg.translation_tolerance
                and abs(step[2]) <= cfg.yaw_tolerance):
            # Information and residuals describe this exact returned pose.
            converged = True
            break
        pose = candidate
    if not converged:
        return result(False, "iteration_limit")
    if rms > cfg.max_rms_residual:
        return result(False, "residual_limit")
    variance = max(cfg.noise_floor ** 2, float(np.sum(weights * residual ** 2) / weights.sum()))
    effective = min(float(weights.sum()), cfg.max_effective_samples)
    information = hessian * effective / weights.sum()
    covariance = variance * np.linalg.inv(information)
    covariance += np.diag([cfg.translation_uncertainty_floor ** 2] * 2
                          + [cfg.yaw_uncertainty_floor ** 2])
    return result(True, "accepted", covariance)
