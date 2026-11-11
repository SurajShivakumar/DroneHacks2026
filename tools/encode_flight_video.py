"""Encode a source-timed flight video from recorded image-sequence manifests.

Each frame is shown for its actual simulation-time spacing (ffconcat
durations), so playback follows simulated time; dropped-frame gaps hold the
previous image and never invent motion. With --onboard, the observer view is
stacked above the onboard stereo/feature-track view, synchronised by source
timestamps. Playback length is not a timing result: native mission telemetry
defines arm/disarm and gate times.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import subprocess


def load_frames(directory, start_s, end_s):
    manifest = next(directory.glob('*.json'))
    frames = json.loads(manifest.read_text())['frames']
    picked = [f for f in frames if start_s <= f['stamp_ns']/1e9 <= end_s]
    if len(picked) < 2:
        raise ValueError(f'fewer than two frames in {directory} for the requested interval')
    return manifest, picked


def write_concat(path, directory, frames, end_s):
    lines = ['ffconcat version 1.0']
    for i, f in enumerate(frames):
        nxt = frames[i+1]['stamp_ns']/1e9 if i+1 < len(frames) else end_s
        lines += [f"file '{(directory/f['file']).resolve()}'", f"duration {max(1e-3, nxt-f['stamp_ns']/1e9):.6f}"]
    lines.append(f"file '{(directory/frames[-1]['file']).resolve()}'")
    path.write_text('\n'.join(lines)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--observer', type=Path, required=True, help='image-sequence dir of the observer camera')
    p.add_argument('--onboard', type=Path, help='optional image-sequence dir (e.g. OpenVINS track history)')
    p.add_argument('--start-s', type=float, required=True)
    p.add_argument('--end-s', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--title', default='')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    ffmpeg = shutil.which('ffmpeg')
    if not ffmpeg:
        raise SystemExit('ffmpeg not found')
    obs_manifest, obs = load_frames(a.observer, a.start_s, a.end_s)
    write_concat(a.output/'observer.ffconcat', a.observer, obs, a.end_s)
    cmd = [ffmpeg, '-y', '-hide_banner', '-f', 'concat', '-safe', '0', '-i', str(a.output/'observer.ffconcat')]
    inputs = {'observer': {'manifest_sha256': sha256(obs_manifest.read_bytes()).hexdigest(), 'frames': len(obs)}}
    if a.onboard:
        on_manifest, on = load_frames(a.onboard, a.start_s, a.end_s)
        write_concat(a.output/'onboard.ffconcat', a.onboard, on, a.end_s)
        cmd += ['-f', 'concat', '-safe', '0', '-i', str(a.output/'onboard.ffconcat')]
        inputs['onboard'] = {'manifest_sha256': sha256(on_manifest.read_bytes()).hexdigest(), 'frames': len(on)}
        graph = ('[0:v]scale=960:-2,setsar=1,fps=15[o];[1:v]scale=960:-2,setsar=1,fps=15[b];'
                 '[o][b]vstack=inputs=2:shortest=1,format=yuv420p[v]')
    else:
        graph = '[0:v]scale=960:-2,setsar=1,fps=15,format=yuv420p[v]'
    cmd += ['-filter_complex', graph, '-map', '[v]', '-c:v', 'libx264', '-crf', '22', '-preset', 'medium',
            '-movflags', '+faststart', str(a.output/'flight.mp4')]
    with open(a.output/'ffmpeg.log', 'w') as log:
        subprocess.run(cmd, check=True, stdout=log, stderr=subprocess.STDOUT)
    video = a.output/'flight.mp4'
    report = {
        'kind': 'source_time_flight_video_not_grade', 'title': a.title,
        'interval_sim_s': [a.start_s, a.end_s], 'inputs': inputs, 'playback_fps': 15,
        'video_sha256': sha256(video.read_bytes()).hexdigest(), 'ffmpeg_command': cmd,
        'limitations': 'Frames held across dropped-frame gaps; no invented motion. Native telemetry, '
                       'not playback length, defines flight and checkpoint timing.',
    }
    (a.output/'video.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ('interval_sim_s', 'inputs', 'video_sha256')}))


if __name__ == '__main__':
    main()
