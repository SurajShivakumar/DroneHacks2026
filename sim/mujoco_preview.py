"""Render a frozen course orbit and onboard RGB/depth; never runs flight control."""

import argparse
from hashlib import sha256
import json
from pathlib import Path
import time

import imageio_ffmpeg
import mujoco
import numpy as np
from PIL import Image, ImageDraw


def record(scene, output, seconds=6, fps=12, width=960, height=540):
    if seconds <= 0 or fps <= 0 or width <= 0 or height <= 0:
        raise ValueError("duration, fps and dimensions must be positive")
    if output.exists():
        raise FileExistsError(f"choose a new output directory: {output}")
    output.mkdir(parents=True)
    started = time.perf_counter()
    model = mujoco.MjModel.from_xml_path(str(scene.resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [0, 0, .45]
    camera.distance, camera.elevation = 9.0, -65
    frames = max(1, round(seconds * fps))
    writer = imageio_ffmpeg.write_frames(str(output / "course-preview.mp4"),
                                        (width, height), fps=fps, macro_block_size=1,
                                        codec="libx264", output_params=["-crf", "23"])
    writer.send(None)
    try:
        with mujoco.Renderer(model, width=width, height=height) as renderer:
            for i in range(frames):
                camera.azimuth = 90 + 60 * i / max(1, frames - 1)
                renderer.update_scene(data, camera)
                image = Image.fromarray(renderer.render())
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, width, 29), fill="black")
                draw.text((10, 9), "MUJOCO COURSE PREVIEW | FROZEN STATE | NO AUTONOMOUS FLIGHT", fill="white")
                if i == 0:
                    image.save(output / "course.png")
                writer.send(np.asarray(image).tobytes())
            renderer.update_scene(data, "rgb")
            Image.fromarray(renderer.render()).save(output / "onboard-rgb.png")
            renderer.enable_depth_rendering()
            renderer.update_scene(data, "rgb")
            np.save(output / "ideal-depth-metres.npy", renderer.render())
    finally:
        writer.close()
    elapsed = time.perf_counter() - started
    report = {"kind": "static_scene_preview_not_a_flight_run", "mujoco": mujoco.__version__,
              "scene_sha256": sha256(scene.read_bytes()).hexdigest(),
              "frames": frames, "video_fps": fps, "wall_seconds": elapsed,
              "rendered_frames_per_wall_second": frames / elapsed,
              "simulation_time_advanced": float(data.time), "actuator_commands": False,
              "finish_time": None, "camera": "observer orbit; not a navigation input",
              "depth": "ideal renderer depth; not measured stereo/OAK performance"}
    (output / "preview.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", type=Path)
    parser.add_argument("--output", type=Path, default=Path("artifacts/mujoco-preview-v1"))
    parser.add_argument("--seconds", type=float, default=6)
    args = parser.parse_args()
    print(json.dumps(record(args.scene, args.output, seconds=args.seconds), indent=2))
