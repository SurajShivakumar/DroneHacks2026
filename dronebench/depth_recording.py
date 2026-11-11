"""Source-stamped depth evidence; no localization or flight commands."""
import json
from hashlib import sha256
from pathlib import Path
import numpy as np


class DepthSequence:
    def __init__(self, output, *, topic, provenance, stride=8, period_ns=500_000_000):
        if (not topic or not provenance or type(stride) is not int or stride<1
                or type(period_ns) is not int or period_ns<1):
            raise ValueError('topic, provenance and positive integer sampling bounds required')
        self.output=Path(output); self.output.mkdir(parents=True,exist_ok=False)
        self.topic=topic; self.provenance=provenance; self.stride=stride
        self.period_ns=period_ns; self.frames=[]; self.last_seen=None

    def append(self, depth_m, stamp_ns, frame_id):
        depth=np.asarray(depth_m)
        if (depth.ndim!=2 or min(depth.shape)<1 or depth.dtype.kind!='f'
                or type(stamp_ns) is not int or stamp_ns<0 or not frame_id):
            raise ValueError('floating metric depth, source stamp and sensor frame required')
        if self.last_seen is not None and stamp_ns<self.last_seen:
            raise ValueError('depth source clock regressed')
        self.last_seen=stamp_ns
        if self.frames and stamp_ns-self.frames[-1]['stamp_ns']<self.period_ns:
            return False
        target=self.output/f'frame_{len(self.frames):06d}.npy'
        # Exact [0,stride,2*stride,...] pixel subsampling, not resized pixels.
        with target.open('xb') as stream:
            np.save(stream,np.asarray(depth[::self.stride,::self.stride],dtype=np.float32),allow_pickle=False)
        self.frames.append(dict(file=target.name,stamp_ns=stamp_ns,frame_id=frame_id,
            source_shape=list(depth.shape),sha256=sha256(target.read_bytes()).hexdigest()))
        return True

    def close(self):
        result=dict(kind='metric_depth_evidence_not_pose_or_flight_qualification',
            topic=self.topic,provenance=self.provenance,units='metres_optical_z',
            pixel_stride=self.stride,pixel_sampling='original pixels [::stride,::stride]; no resampling',
            period_ns=self.period_ns,requires_separate_camera_calibration=True,frames=self.frames)
        with (self.output/'sequence.json').open('x',encoding='utf-8') as stream:
            json.dump(result,stream,indent=2,allow_nan=False)
        return result
