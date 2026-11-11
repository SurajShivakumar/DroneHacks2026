"""Pure observed-event sequencing; no perception, flight commands or approval.

All timestamps use one caller-selected clock (ROS simulation time in SITL).
The adapter must establish each event from fresh sensor/estimator evidence.
Full clearance is an explicit input contract, not inferred here from a point pose.
"""

from dataclasses import dataclass
from math import isfinite

from dronebench.experiments import Preferences


class MissionError(ValueError):
    """An event cannot advance the mission."""


@dataclass(frozen=True)
class Observation:
    event: str
    stamp: float
    evidence: str
    confidence: float = 1.0
    ambiguous: bool = False
    full_clearance: bool = False

    def __post_init__(self):
        if not isinstance(self.event, str) or not self.event:
            raise ValueError("event name required")
        if not isinstance(self.evidence, str) or not self.evidence.strip():
            raise ValueError("source evidence reference required")
        if not isfinite(self.stamp) or self.stamp < 0:
            raise ValueError("invalid source timestamp")
        if not isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("confidence must be in [0, 1]")
        if type(self.ambiguous) is not bool or type(self.full_clearance) is not bool:
            raise ValueError("observation flags must be boolean")


@dataclass(frozen=True)
class Step:
    state: str
    event: str
    clearance: bool = False


class Mission:
    """Frozen route and ordered evidence log, independent of the command owner.

    `ready` is evidence of prechecks only, never permission to arm. `landed_on_pad`
    requires target and landed evidence; `disarmed` separately requires vehicle
    status. Return precheck/takeoff is explicit after outbound disarm. The return
    route is a separate barrier crossing, not an assumed reverse checkpoint list.
    Timeout/abort enters FAULT and preserves completed outbound evidence.
    """

    def __init__(self, preferences=Preferences(), *, max_age=0.5,
                 min_confidence=0.9, stage_timeout=60.0):
        if not isinstance(preferences, Preferences):
            raise ValueError("validated Preferences required")
        for name, value in (("max_age", max_age), ("stage_timeout", stage_timeout)):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if not isfinite(min_confidence) or not 0 <= min_confidence <= 1:
            raise ValueError("invalid confidence threshold")
        self._preferences = preferences
        self.max_age, self.min_confidence = max_age, min_confidence
        self.stage_timeout = stage_timeout
        self._steps = (
            Step("PRECHECK", "ready"), Step("TAKEOFF", "airborne"),
            Step("FIRST_CORRIDOR", "corridor", True),
            Step("MAZE_EXIT", "maze_exit", True), Step("TUNNEL", "tunnel", True),
            Step("CHICANE", preferences.chicane, True),
            Step("LANDING", "landed_on_pad", True), Step("DISARM", "disarmed"),
        )
        if preferences.return_enabled:
            self._steps += (Step("RETURN_PRECHECK", "return_ready"),
                            Step("RETURN_TAKEOFF", "return_airborne"),
                            Step("CROSS_BARRIER", "return_barrier", True),
                            Step("RETURN_LAND", "return_landed_on_pad", True),
                            Step("RETURN_DISARM", "return_disarmed"))
        self._index = 0
        self._accepted = []
        self._last_now = None
        self._entered_at = None
        self._fault = None

    @property
    def preferences(self):
        return self._preferences

    @property
    def state(self):
        return "FAULT" if self._fault else ("COMPLETE" if self._index == len(self._steps)
                                           else self._steps[self._index].state)

    @property
    def expected_event(self):
        return None if self.state in ("FAULT", "COMPLETE") else self._steps[self._index].event

    @property
    def accepted(self):
        return tuple(self._accepted)

    @property
    def outbound_complete(self):
        return any(event.event == "disarmed" for event in self._accepted)

    @property
    def fault(self):
        return self._fault

    def tick(self, now):
        """Check a stage timeout even when no observations arrive."""
        if not isfinite(now) or now < 0:
            raise MissionError("invalid current timestamp")
        if self._last_now is not None and now < self._last_now:
            raise MissionError("clock regressed; create a new mission after reset")
        self._last_now = now
        if self._entered_at is None:
            self._entered_at = now
        if self.state not in ("COMPLETE", "FAULT") and now - self._entered_at > self.stage_timeout:
            self.abort("stage timeout")
        return self.state

    def observe(self, observation, *, now):
        self.tick(now)
        if self.state in ("FAULT", "COMPLETE"):
            raise MissionError(f"mission is {self.state}")
        if not isinstance(observation, Observation):
            raise MissionError("Observation required")
        age = now - observation.stamp
        if age < 0 or age > self.max_age:
            raise MissionError("future or stale observation")
        if self._accepted and observation.stamp <= self._accepted[-1].stamp:
            raise MissionError("source timestamp did not advance")
        if observation.ambiguous or observation.confidence < self.min_confidence:
            raise MissionError("ambiguous or low-confidence observation")
        step = self._steps[self._index]
        if observation.event != step.event:
            raise MissionError(f"expected {step.event}; cannot skip checkpoints")
        if step.clearance and not observation.full_clearance:
            raise MissionError("whole-aircraft clearance evidence required")
        self._accepted.append(observation)
        self._index += 1
        self._entered_at = now
        return self.state

    def abort(self, reason):
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("abort reason required")
        if self.state != "COMPLETE":
            self._fault = reason
