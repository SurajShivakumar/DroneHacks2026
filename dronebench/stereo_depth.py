"""Offline metric depth from calibrated rectified mono8 stereo; no hole filling.

The caller must verify physical calibration, synchronization, rectification,
matching image/camera-info dimensions and optical-frame convention. Inputs must
have equal rectified principal points (or have their disparity offset removed).
Positive disparity is x_left - x_right; optical Z = fx * baseline / disparity.
This module reads neither rendered ideal depth nor simulator/world state.
Independent local alternative-disparity costs reject comparable periodic
matches, even when SGBM smoothness and left/right checks agree on one phase.
These checks still cannot prove a match: validity is a matching diagnostic,
never estimator/flight readiness.
"""

from dataclasses import asdict, dataclass
from math import isfinite

import numpy as np


@dataclass(frozen=True)
class StereoDepthConfig:
    fx: float
    baseline_m: float
    min_depth_m: float = .25
    max_depth_m: float = 8.
    min_disparity: int = 0
    num_disparities: int = 128
    block_size: int = 5
    uniqueness_ratio: int = 15
    lr_max_error_px: float = .75
    texture_window: int = 7
    min_texture_std: float = 5.
    max_photometric_error: float = 20.
    ambiguity_exclusion_px: float = 1.5
    ambiguity_relative_margin: float = .15
    ambiguity_absolute_margin: float = 2.
    speckle_window_size: int = 50
    speckle_range: int = 1

    def __post_init__(self):
        for name in ("fx", "baseline_m", "min_depth_m", "max_depth_m",
                     "lr_max_error_px", "min_texture_std", "max_photometric_error",
                     "ambiguity_exclusion_px", "ambiguity_relative_margin", "ambiguity_absolute_margin"):
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError("depth limits must be strictly ordered")
        for name in ("min_disparity", "num_disparities", "block_size", "uniqueness_ratio",
                     "texture_window", "speckle_window_size", "speckle_range"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if self.min_disparity < 0:
            raise ValueError("left min_disparity must be nonnegative")
        if self.num_disparities <= 0 or self.num_disparities % 16:
            raise ValueError("num_disparities must be a positive multiple of 16")
        if self.min_disparity + self.num_disparities > 2048:
            raise ValueError("disparity range exceeds signed fixed-point output")
        if not 3 <= self.block_size <= 11 or self.block_size % 2 != 1:
            raise ValueError("block_size must be odd and in [3,11]")
        if self.texture_window < 3 or self.texture_window % 2 != 1:
            raise ValueError("texture_window must be odd and >= 3")
        if not 1 <= self.uniqueness_ratio <= 100:
            raise ValueError("uniqueness_ratio must be in [1,100]")
        if self.ambiguity_exclusion_px < 1 or self.ambiguity_exclusion_px >= self.num_disparities / 2:
            raise ValueError("ambiguity_exclusion_px must be >=1 and less than half the search range")
        if self.speckle_window_size < 0 or self.speckle_range <= 0:
            raise ValueError("speckle_window_size must be nonnegative and speckle_range positive")
        with np.errstate(over="ignore", invalid="ignore"):
            fb = self.fx * self.baseline_m
        if not isfinite(fb) or fb <= 0:
            raise ValueError("fx * baseline_m must be finite and positive")
        if (fb / self.max_depth_m < self.min_disparity
                or fb / self.min_depth_m > self.min_disparity + self.num_disparities - 1):
            raise ValueError("disparity search must cover the requested metric depth interval")


@dataclass(frozen=True)
class StereoDepthResult:
    depth_m: np.ndarray
    valid: np.ndarray
    disparity_px: np.ndarray
    diagnostics: dict


def _independent_ambiguity_check(left, right, selected_disparity, cfg, cv2):
    """Stream independent constant-disparity patch SAD costs using O(HW) memory.

    A chosen phase must beat every separated integer disparity by an absolute
    plus relative margin. Nearby integer disparities cover subpixel rounding;
    they are not competing phases. Searches include zero disparity when it is
    configured: the metric range must not hide an indistinguishable background.
    Unchecked/missing support fails closed. No smoothness prior is used here.
    """
    height, width = left.shape
    radius = cfg.block_size // 2
    left_float, right_float = left.astype(np.float32), right.astype(np.float32)
    selected_cost = np.full(left.shape, np.inf, dtype=np.float32)
    alternative_cost = np.full(left.shape, np.inf, dtype=np.float32)
    for disparity in range(cfg.min_disparity, cfg.min_disparity + cfg.num_disparities):
        if disparity + 2 * radius >= width:
            continue
        # Every candidate shifts the original right image by one constant
        # disparity. Warping by the chosen disparity field would conceal local
        # alternatives and mix smoothness decisions into this independent cost.
        error = np.full(left.shape, 255., dtype=np.float32)
        error[:, disparity:] = np.abs(left_float[:, disparity:] - right_float[:, :width - disparity])
        cost = cv2.boxFilter(error, -1, (cfg.block_size, cfg.block_size), normalize=True)
        cost[:radius] = np.inf
        cost[height - radius:] = np.inf
        cost[:, :disparity + radius] = np.inf
        cost[:, width - radius:] = np.inf
        separation = np.abs(selected_disparity - disparity)
        near = separation <= 1.
        alternative = separation > cfg.ambiguity_exclusion_px
        np.minimum(selected_cost, cost, out=selected_cost, where=near)
        np.minimum(alternative_cost, cost, out=alternative_cost, where=alternative)
    checked = np.isfinite(selected_cost) & np.isfinite(alternative_cost)
    gap = np.full(left.shape, np.nan, dtype=np.float32)
    np.subtract(alternative_cost, selected_cost, out=gap, where=checked)
    unique = checked & (gap > cfg.ambiguity_absolute_margin + cfg.ambiguity_relative_margin * selected_cost)
    return unique, gap


def rectified_stereo_depth(left_mono8, right_mono8, config):
    """Compute checked left optical-Z depth, leaving every invalid pixel NaN.

    OpenCV is an optional runtime dependency. Returns explicit stage counts and
    configuration without any quality/readiness claim. Image dimensions that
    cannot support the full requested disparity search are rejected, not resized
    or silently narrowed. Output disparity is NaN wherever depth is invalid.
    """
    if not isinstance(config, StereoDepthConfig):
        raise ValueError("config must be StereoDepthConfig")
    cfg = config
    left, right = np.asarray(left_mono8), np.asarray(right_mono8)
    if (left.ndim != 2 or right.ndim != 2 or left.shape != right.shape
            or left.dtype != np.uint8 or right.dtype != np.uint8):
        raise ValueError("matching HxW rectified mono8 (uint8) images required")
    height, width = left.shape
    margin = max(cfg.block_size, cfg.texture_window) // 2
    if (height < 2 * margin + 1
            or width <= cfg.min_disparity + cfg.num_disparities + 2 * margin):
        raise ValueError("images too small for requested disparity and support windows")
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("rectified_stereo_depth requires the optional OpenCV cv2 dependency") from exc

    def matcher(minimum):
        return cv2.StereoSGBM_create(
            minDisparity=minimum, numDisparities=cfg.num_disparities,
            blockSize=cfg.block_size, P1=8 * cfg.block_size ** 2,
            P2=32 * cfg.block_size ** 2, uniquenessRatio=cfg.uniqueness_ratio,
            # Explicit subpixel left/right checking below is authoritative.
            disp12MaxDiff=1, preFilterCap=31,
            speckleWindowSize=cfg.speckle_window_size, speckleRange=cfg.speckle_range,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    left = np.ascontiguousarray(left)
    right = np.ascontiguousarray(right)
    disparity_left = matcher(cfg.min_disparity).compute(left, right).astype(np.float32) / 16.
    # Swapped matching searches x_right - x_left, i.e. the NEGATIVE left range.
    right_minimum = -cfg.min_disparity - cfg.num_disparities + 1
    disparity_right = matcher(right_minimum).compute(right, left).astype(np.float32) / 16.
    left_ok = ((disparity_left >= cfg.min_disparity)
               & (disparity_left <= cfg.min_disparity + cfg.num_disparities - 1)
               & (disparity_left > 0))
    right_ok = ((disparity_right >= right_minimum)
                & (disparity_right <= -cfg.min_disparity)
                & (disparity_right < 0))
    yy, xx = np.indices(left.shape)
    x_right = xx - disparity_left
    x0 = np.floor(x_right).astype(np.int32)
    x1 = x0 + 1
    in_bounds = ((x0 >= margin) & (x1 < width - margin) & (xx >= margin)
                 & (xx < width - margin) & (yy >= margin) & (yy < height - margin))
    ix0, ix1 = np.clip(x0, 0, width - 1), np.clip(x1, 0, width - 1)
    fraction = x_right - x0
    right_at_match = ((1 - fraction) * disparity_right[yy, ix0]
                      + fraction * disparity_right[yy, ix1])
    lr_ok = (left_ok & in_bounds & right_ok[yy, ix0] & right_ok[yy, ix1]
             & (np.abs(disparity_left + right_at_match) <= cfg.lr_max_error_px))

    def texture_ok(image):
        pixels = image.astype(np.float32)
        window = (cfg.texture_window, cfg.texture_window)
        mean = cv2.boxFilter(pixels, -1, window, normalize=True)
        square_mean = cv2.boxFilter(pixels * pixels, -1, window, normalize=True)
        return np.maximum(square_mean - mean * mean, 0) >= cfg.min_texture_std ** 2

    left_texture, right_texture = texture_ok(left), texture_ok(right)
    texture = left_texture & right_texture[yy, ix0] & right_texture[yy, ix1]
    # Reciprocal random mismatches can survive LR checking. Require a matching
    # intensity patch too; differing exposure/blur can conservatively lose data.
    warped_right = cv2.remap(right.astype(np.float32), x_right.astype(np.float32),
                             yy.astype(np.float32), cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    error = np.abs(left.astype(np.float32) - warped_right)
    patch_error = cv2.boxFilter(error, -1, (cfg.block_size, cfg.block_size), normalize=True)
    photometric = patch_error <= cfg.max_photometric_error
    with np.errstate(divide="ignore", invalid="ignore"):
        candidate_depth = (cfg.fx * cfg.baseline_m) / disparity_left
    depth_range = (np.isfinite(candidate_depth) & (candidate_depth >= cfg.min_depth_m)
                   & (candidate_depth <= cfg.max_depth_m))
    before_ambiguity = lr_ok & texture & photometric & depth_range
    unique, _ = _independent_ambiguity_check(left, right, disparity_left, cfg, cv2)
    valid = before_ambiguity & unique
    depth = np.full(left.shape, np.nan, dtype=np.float32)
    disparity = np.full(left.shape, np.nan, dtype=np.float32)
    depth[valid] = candidate_depth[valid]
    disparity[valid] = disparity_left[valid]
    diagnostics = {
        "configuration": asdict(cfg), "opencv_version": cv2.__version__,
        "right_min_disparity": right_minimum, "pixels": int(left.size),
        "left_match_pixels": int(left_ok.sum()), "lr_consistent_pixels": int(lr_ok.sum()),
        "textured_lr_pixels": int((lr_ok & texture).sum()), "valid_pixels": int(valid.sum()),
        "photometric_textured_lr_pixels": int((lr_ok & texture & photometric).sum()),
        "pre_ambiguity_valid_pixels": int(before_ambiguity.sum()),
        "ambiguity_rejected_pixels": int((before_ambiguity & ~unique).sum()),
        "ambiguity_method": "independent_streamed_constant_disparity_patch_SAD",
        "invalid_pixels": int((~valid).sum()), "valid_fraction": float(valid.mean()),
        "scope": "offline_rectified_stereo_matching_only",
    }
    return StereoDepthResult(depth, valid, disparity, diagnostics)
