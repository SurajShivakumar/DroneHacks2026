"""Read-only stereo observation contracts; no ROS or flight authority.

Source pairing is causal. A declared map/VIO calibration is required; this
module never discovers an alignment from simulator state or commanded motion.
"""
from collections import deque
import numpy as np


def decode_mono8(image):
    if (image.encoding.lower() not in ('mono8','8uc1') or image.width<1 or image.height<1
            or image.step<image.width or len(image.data)!=image.height*image.step):
        raise ValueError('valid mono8 dimensions, stride and payload required')
    return np.frombuffer(image.data,dtype=np.uint8).reshape(image.height,image.step)[:,:image.width].copy()


def body_height_from_range(distance,rotation,offset_flu,axis_flu):
    R=np.asarray(rotation,float); offset=np.asarray(offset_flu,float); axis=np.asarray(axis_flu,float)
    if (R.shape!=(3,3) or offset.shape!=(3,) or axis.shape!=(3,)
            or not np.isfinite([distance,*R.ravel(),*offset,*axis]).all() or distance<=0
            or not np.allclose(R.T@R,np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(R),1)
            or not np.isclose(np.linalg.norm(axis),1)):
        raise ValueError('finite range and calibrated proper body/range transforms required')
    direction=R@axis
    if direction[2]>-.7:raise ValueError('range ray insufficiently downward')
    height=-(R@offset)[2]-distance*direction[2]
    if height<=0:raise ValueError('range projects body below floor')
    return float(height)


class CausalSamples:
    """Bounded source history; resets latch a fault, never silently reanchor."""
    def __init__(self,capacity=1000):
        self.rows=deque(maxlen=capacity); self.fault=None

    def append(self,stamp,payload,arrival):
        if not np.isfinite([stamp,arrival]).all() or stamp<=0:
            raise ValueError('finite positive source stamp required')
        if self.fault:raise ValueError(self.fault)
        if self.rows and stamp<=self.rows[-1][0]:
            self.fault='nonadvancing source/reset; restart observer with reviewed initialization'
            raise ValueError(self.fault)
        self.rows.append((stamp,payload,arrival))

    def at(self,stamp,now,*,max_skew=.05,max_arrival_age=.5):
        if self.fault:raise ValueError(self.fault)
        if not np.isfinite([stamp,now,max_skew,max_arrival_age]).all() or min(max_skew,max_arrival_age)<=0:
            raise ValueError('finite positive timing bounds required')
        for source,payload,arrival in reversed(self.rows):
            if source<=stamp:
                if stamp-source>max_skew or not 0<=now-arrival<=max_arrival_age:
                    raise ValueError('causal sample stale by source or arrival')
                return source,payload
        raise ValueError('no causal sample')
