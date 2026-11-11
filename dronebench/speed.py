"""Acceleration/braking profile for an already collision-checked sampled path.

This is a planning component, not an autopilot, estimator or collision checker.
Units are metres, seconds and radians. Samples must cover the swept path;
clearance/visibility are inputs from a future perception implementation.
"""

from dataclasses import dataclass
from math import isfinite, sqrt


@dataclass(frozen=True)
class Limits:
    cruise: float = 1.0
    acceleration: float = 0.5
    braking: float = 0.5
    lateral: float = 0.5
    latency: float = 0.2
    margin: float = 0.3

    def __post_init__(self):
        for name in ("cruise", "acceleration", "braking", "lateral"):
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("latency", "margin"):
            value = getattr(self, name)
            if not isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")


@dataclass(frozen=True)
class Sample:
    distance: float
    curvature: float
    free_distance: float
    confidence: float = 1.0
    speed_cap: float | None = None

    def __post_init__(self):
        if not all(isfinite(x) for x in (
            self.distance, self.curvature, self.free_distance, self.confidence
        )):
            raise ValueError("path values must be finite")
        if self.distance < 0 or self.free_distance < 0:
            raise ValueError("distances must be nonnegative")
        if not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be between zero and one")
        if self.speed_cap is not None:
            if not isfinite(self.speed_cap) or self.speed_cap < 0:
                raise ValueError("speed_cap must be finite and nonnegative")


def speed_profile(samples, limits=Limits(), start_speed=0.0, end_speed=0.0):
    """Return sampled speed bounds, with fixed feasible endpoint speeds.

    Uses constant tangential acceleration between samples. Confidence scaling
    is a heuristic, not a calibrated probability or a safety guarantee.
    Jerk, coupled thrust limits, tracking and collision validation remain TODO.
    """
    samples = list(samples)
    if len(samples) < 2:
        raise ValueError("at least two samples are required")
    for speed in (start_speed, end_speed):
        if not isfinite(speed) or speed < 0:
            raise ValueError("endpoint speeds must be finite and nonnegative")
    if any(b.distance <= a.distance for a, b in zip(samples, samples[1:])):
        raise ValueError("sample distances must strictly increase")
    speeds = []
    for sample in samples:
        # v*t + v^2/(2*b) <= observed free distance minus margin.
        available = max(0.0, sample.free_distance - limits.margin)
        bt = limits.braking * limits.latency
        stopping_cap = sqrt(bt * bt + 2 * limits.braking * available) - bt
        curvature_cap = (
            sqrt(limits.lateral / abs(sample.curvature))
            if sample.curvature else limits.cruise
        )
        cap = limits.cruise if sample.speed_cap is None else sample.speed_cap
        speeds.append(min(limits.cruise, curvature_cap, stopping_cap, cap)
                      * sample.confidence)
    if start_speed > speeds[0] + 1e-9 or end_speed > speeds[-1] + 1e-9:
        raise ValueError("endpoint speed exceeds a local bound")
    speeds[-1] = end_speed
    for i in range(len(samples) - 2, -1, -1):
        ds = samples[i + 1].distance - samples[i].distance
        speeds[i] = min(speeds[i], sqrt(speeds[i + 1] ** 2 + 2 * limits.braking * ds))
    if start_speed > speeds[0] + 1e-9:
        raise ValueError("insufficient distance to brake from start_speed")
    speeds[0] = start_speed
    for i in range(1, len(samples)):
        ds = samples[i].distance - samples[i - 1].distance
        speeds[i] = min(speeds[i], sqrt(speeds[i - 1] ** 2 + 2 * limits.acceleration * ds))
    if abs(speeds[-1] - end_speed) > 1e-9:
        raise ValueError("insufficient distance to reach end_speed")
    return speeds


def ideal_duration(samples, speeds):
    """Ideal profile time, explicitly NOT a measured flight/finish time."""
    samples, speeds = list(samples), list(speeds)
    if len(samples) != len(speeds) or len(samples) < 2:
        raise ValueError("matching samples and speeds are required")
    total = 0.0
    for a, b, va, vb in zip(samples, samples[1:], speeds, speeds[1:]):
        if (not isfinite(va) or not isfinite(vb) or min(va, vb) < 0
                or b.distance <= a.distance):
            raise ValueError("invalid segment")
        if va + vb == 0:
            raise ValueError("zero-speed segment cannot be traversed")
        total += 2 * (b.distance - a.distance) / (va + vb)
    return total
