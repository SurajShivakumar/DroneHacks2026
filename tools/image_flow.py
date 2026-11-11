"""Translational image-flow model for speed limiting (offline, published map).

For a camera at `position` with heading `yaw` moving along unit direction `u`,
each image ray hitting a surface at range r sweeps at angular rate
|u - (u.d)d| / r per m/s (d = unit ray). In pixels per frame that is
FX * rate / FPS. Fast KLT tracking loses features when most of the view moves
too far between frames (v60/v61: sideways view of a near wall at speed).
"""
import numpy as np

import prepare_heading_reference as phr

FPS = 15.


def flow_px_per_frame_per_mps(geometry, rays, position, yaw, u, quantile=.5, max_range=9.):
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    world = rays@rot.T
    hits = geometry.cast(np.asarray(position, float), world)
    t = hits.t
    ok = np.isfinite(t)
    if not ok.any():
        return 0.
    norm = np.linalg.norm(world[ok], axis=1)
    r = t[ok]*norm
    d = world[ok]/norm[:, None]
    keep = r <= max_range
    if not keep.any():
        return 0.
    d, r = d[keep], r[keep]
    u = np.asarray(u, float)/max(np.linalg.norm(u), 1e-9)
    lateral = np.linalg.norm(u[None, :]-(d@u)[:, None]*d, axis=1)
    rate = lateral/np.maximum(r, .05)
    return float(phr.FX*np.quantile(rate, quantile)/FPS)
