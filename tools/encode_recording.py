"""Encode an offline ImageSequence as a fixed-rate viewing aid, not a race result.

Usage: python tools/encode_recording.py SEQUENCE_DIR NEW_OUTPUT_DIR --fps 15
The sequence must already contain sequence.json and its hashed PNG frames.
"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.recording import encode_preview


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sequence", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fps", type=float, default=15.)
    parser.add_argument("--ffmpeg")
    args = parser.parse_args()
    encode_preview(args.sequence, args.output, fps=args.fps, ffmpeg=args.ffmpeg)
