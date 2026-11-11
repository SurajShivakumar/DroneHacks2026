"""Offline-only CLI. No ROS, network, model loading or flight commands."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .experiments import Preferences, STRATEGIES, create_manifest
from .speed import Limits, Sample, ideal_duration, speed_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("strategies")
    prepare = commands.add_parser("prepare")
    prepare.add_argument("--strategy", choices=STRATEGIES, required=True)
    prepare.add_argument("--preferences", type=Path, required=True)
    prepare.add_argument("--input", action="append", type=Path, required=True)
    prepare.add_argument("--output", type=Path, required=True)
    profile = commands.add_parser("profile")
    profile.add_argument("path", type=Path)
    profile.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.command == "strategies":
            print(json.dumps(STRATEGIES, indent=2))
            return
        if args.command == "prepare":
            result = create_manifest(args.strategy, Preferences.load(args.preferences),
                                     [args.preferences, *args.input])
        else:
            raw = json.loads(args.path.read_text(encoding="utf-8"))
            limits = Limits(**raw["limits"])
            samples = [Sample(**sample) for sample in raw["samples"]]
            speeds = speed_profile(samples, limits, raw.get("start_speed", 0.0),
                                   raw.get("end_speed", 0.0))
            result = {
                "kind": "offline_speed_profile_not_flight",
                "limits": asdict(limits),
                "ideal_profile_seconds": ideal_duration(samples, speeds),
                "samples": [{**asdict(s), "speed": v} for s, v in zip(samples, speeds)],
            }
        # Never silently overwrite evidence from a previous invocation.
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(args.output)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    main()
