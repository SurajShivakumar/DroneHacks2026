"""Read-only stream timing diagnostics, not a flight-readiness decision."""

from dataclasses import dataclass, field
from math import isfinite
from numbers import Integral, Real


def json_safe(value):
    """Preserve unknown/nonfinite ROS values explicitly without invalid JSON."""
    if isinstance(value,bool):
        return value
    if isinstance(value,Integral):
        return int(value)
    if isinstance(value,Real):
        return float(value) if isfinite(value) else str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


@dataclass
class StreamStats:
    count: int = 0
    first_receive: float | None = None
    last_receive: float | None = None
    first_source: float | None = None
    last_source: float | None = None
    advances: int = 0
    regressions: int = 0
    latest_fields: dict = field(default_factory=dict)

    def observe(self, received, source_stamp=None, fields=None):
        if not isfinite(received):
            raise ValueError("receive timestamp must be finite")
        if self.last_receive is not None and received < self.last_receive:
            raise ValueError("receive clock must be monotonic")
        if source_stamp is not None and not isfinite(source_stamp):
            raise ValueError("source timestamp must be finite")
        self.count += 1
        if self.first_receive is None:
            self.first_receive = received
        self.last_receive = received
        if source_stamp is not None:
            if self.first_source is None:
                self.first_source = source_stamp
            if self.last_source is not None:
                self.advances += source_stamp > self.last_source
                self.regressions += source_stamp < self.last_source
            self.last_source = source_stamp
        self.latest_fields = dict(fields or {})

    def summary(self, now):
        if not isfinite(now) or (self.last_receive is not None and now < self.last_receive):
            raise ValueError("invalid report clock")
        interval = (self.last_receive - self.first_receive) if self.count > 1 else 0
        return {
            "messages": self.count,
            "receive_hz_wall": (self.count - 1) / interval if interval > 0 else None,
            "last_receive_age_wall_s": now - self.last_receive if self.count else None,
            "source_first_s": self.first_source,
            "source_last_s": self.last_source,
            "source_advances": self.advances,
            "source_regressions": self.regressions,
            "source_clock_observed_advancing": self.advances > 0,
            "latest_fields": self.latest_fields,
        }
