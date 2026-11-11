"""Offline image evidence. Source timestamps, never video duration, describe time.

This stores a single stream in a named clock domain. It does not evaluate course
completion. Fixed-rate video is a viewing aid and can compress gaps or pauses.
"""

import json
import math
from pathlib import Path
import shutil
import subprocess

from dronebench.experiments import digest


class ImageSequence:
    """Exclusive evidence directory accepting strictly increasing source stamps."""

    def __init__(self, output, *, topic, clock_domain, provenance):
        if not topic or not clock_domain or not isinstance(provenance, dict):
            raise ValueError("topic, clock domain and provenance are required")
        # Validate serializability before creating an output.
        json.dumps(provenance, allow_nan=False)
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.frames = []
        self.closed = False
        self.metadata = {"schema_version": 1, "kind": "offline_image_evidence",
                         "topic": topic, "clock_domain": clock_domain,
                         "provenance": provenance, "finish_time_seconds": None,
                         "completion_evaluated": False}

    def append(self, image, stamp_ns):
        from PIL import Image
        if self.closed:
            raise ValueError("sequence is closed")
        if type(stamp_ns) is not int or stamp_ns < 0:
            raise ValueError("stamp_ns must be a nonnegative integer")
        if self.frames and stamp_ns <= self.frames[-1]["stamp_ns"]:
            raise ValueError("duplicate or regressing source timestamp")
        source = Path(image)
        with Image.open(source) as frame:
            frame.load()
            if frame.format != "PNG":
                raise ValueError("lossless PNG input is required")
            size = list(frame.size)
        if self.frames and size != self.frames[0]["size"]:
            raise ValueError("frame dimensions changed")
        target = self.output / f"frame_{len(self.frames):06d}.png"
        with source.open("rb") as reader, target.open("xb") as writer:
            shutil.copyfileobj(reader, writer)
        self.frames.append({"file": target.name, "stamp_ns": stamp_ns,
                            "sha256": digest(target), "size": size})

    def close(self):
        if self.closed:
            raise ValueError("sequence is closed")
        if not self.frames:
            raise ValueError("cannot finalize an empty sequence")
        result = dict(self.metadata, frames=self.frames,
                      source_span_seconds=(self.frames[-1]["stamp_ns"] -
                                           self.frames[0]["stamp_ns"]) / 1e9)
        with (self.output / "sequence.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
        self.closed = True
        return result


def encode_preview(sequence, output, *, fps=15., ffmpeg=None):
    """Encode a fixed-rate viewing aid, retaining the original timestamp manifest.

Output is an exclusive directory; failures leave logs and inputs untouched.
No interpolation of source timestamps and no mission completion inference occur.
"""
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be finite and positive")
    sequence = Path(sequence)
    manifest = json.loads((sequence / "sequence.json").read_text(encoding="utf-8"))
    frames = manifest["frames"]
    if not frames:
        raise ValueError("empty sequence")
    last = -1
    for index, frame in enumerate(frames):
        if frame["file"] != f"frame_{index:06d}.png":
            raise ValueError("unexpected frame filename")
        if type(frame["stamp_ns"]) is not int or frame["stamp_ns"] <= last:
            raise ValueError("invalid frame timestamp")
        last = frame["stamp_ns"]
        if digest(sequence / frame["file"]) != frame["sha256"]:
            raise ValueError("frame hash mismatch")
    executable = ffmpeg or shutil.which("ffmpeg")
    if not executable:
        raise RuntimeError("ffmpeg is unavailable; supply --ffmpeg")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    command = [str(executable), "-nostdin", "-n", "-framerate", str(fps),
               "-start_number", "0", "-i", str(sequence.resolve() / "frame_%06d.png"),
               "-frames:v", str(len(frames)), "-c:v", "libx264", "-pix_fmt", "yuv420p",
               "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", str((output / "preview.mp4").resolve())]
    with (output / "ffmpeg.log").open("xb") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    result = {"kind": "fixed_rate_preview_not_mission_result", "playback_fps": fps,
              "frame_count": len(frames), "source_manifest_sha256": digest(sequence / "sequence.json"),
              "source_timestamps_ns": [frame["stamp_ns"] for frame in frames],
              "video_sha256": digest(output / "preview.mp4"), "finish_time_seconds": None,
              "encoder_command": command}
    (output / "preview.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    return result
