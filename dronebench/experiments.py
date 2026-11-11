"""Strict pre-run preferences and provenance; preparation never implies flight."""

from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from hashlib import sha256
import json
from pathlib import Path


STRATEGIES = {
    "classical": {
        "kind": "algorithm",
        "status": "native_openvins_sim_completed_organizer_pipeline_pending",
        "evidence": "benchmarks/gb10-sky-beta-course-v83/README.md",
    },
    "teammate_ppo": {
        "kind": "rl_policy", "status": "numpy_rollout_reproduced_px4_flight_pending",
        "source": "https://github.com/jakefromstatefarm00/dronehacks2026",
        "source_commit": "a03b7a6a7e164b37e095bbe7555451235f675efe",
        "checkpoint": "dronehacks_rl.zip:runs/ppo/policy.onnx",
    },
    "openfly": {
        "kind": "vla", "status": "offline_inference_measured_flight_adapter_unqualified",
        "status_as_of": "2026-11-05",
        "source": "https://github.com/SHAILAB-IPEC/OpenFly-Platform",
        "checkpoint": "IPEC-COMMUNITY/openfly-agent-7b",
    },
    "aerovla": {
        "kind": "vla", "status": "offline_adapter_implemented_base_not_staged",
        "status_as_of": "2026-11-05",
        "source": "https://github.com/XuPeng23/AeroVLA",
        "checkpoint": "XuPeng23/AerialVLA (adapter; matching base also required)",
    },
    "openfly_v2": {
        "kind": "vla", "status": "blocked_unreleased_code_and_weights",
        "status_as_of": "2026-11-05",
        "source": "https://github.com/Ideal-ljl/AeroBrain",
        "checkpoint": None,
    },
    "world_model": {
        "kind": "predictive_model",
        "status": "offline_component_flight_pending",
        "generality": "observed_map_and_goal_list_are_inputs",
    },
}


@dataclass(frozen=True)
class Preferences:
    instruction: str = "Follow checkpoints in order; use Gate B; land and disarm."
    chicane: str = "gate_b"
    return_enabled: bool = False
    speed_scale: float = 1.0

    def __post_init__(self):
        if not isinstance(self.instruction, str) or not self.instruction.strip():
            raise ValueError("instruction is required")
        if self.chicane not in ("gate_b", "window"):
            raise ValueError("chicane must be gate_b or window")
        if type(self.return_enabled) is not bool:
            raise ValueError("return_enabled must be boolean")
        if type(self.speed_scale) not in (int, float) or not 0 < self.speed_scale <= 1:
            raise ValueError("speed_scale must be in (0, 1]; increase qualified limits separately")

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))


def digest(path):
    h = sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def create_manifest(strategy, preferences, inputs):
    if strategy not in STRATEGIES:
        raise ValueError("unknown strategy")
    return {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "status": "prepared_not_flown",
        "strategy": strategy,
        "implementation": STRATEGIES[strategy],
        "preferences": asdict(preferences),
        "inputs": [{"name": Path(p).name, "sha256": digest(p)} for p in inputs],
        "finetuned": False,
        "flight_approved": False,
        "result": None,
        "recording": None,
    }
