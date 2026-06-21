"""
Color-based circular-target detection (OpenCV, no ROS).

Pure functions so the detection logic can be unit tested without rclpy. Finds
brightly colored circular blobs (e.g. a red ball) via HSV thresholding plus
contour analysis, returning circles in pixel coordinates. This is a cheap,
training-free alternative to a neural detector for a single known-color target.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class HsvBand:
    """One inclusive HSV threshold band (OpenCV hue is 0-179)."""

    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int

    def lower(self) -> np.ndarray:
        return np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)

    def upper(self) -> np.ndarray:
        return np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)


@dataclass
class ColorDetectParams:
    # Red wraps the hue circle, so two bands are used by default.
    bands: list[HsvBand] = field(default_factory=list)
    min_radius_px: int = 6
    max_radius_px: int = 100000
    blur_ksize: int = 5          # odd; <3 disables blur
    morph_ksize: int = 5         # <1 disables morphology
    min_fill_ratio: float = 0.55  # contour_area / enclosing_circle_area; rejects non-round blobs


@dataclass
class CircleDetection:
    cx: float
    cy: float
    radius: float
    confidence: float
    fill_ratio: float
    area: float


def default_red_params() -> ColorDetectParams:
    """Two-band red (low and high hue) with moderate saturation/value floors."""
    return ColorDetectParams(
        bands=[
            HsvBand(0, 10, 120, 255, 70, 255),
            HsvBand(170, 179, 120, 255, 70, 255),
        ]
    )


def build_mask(bgr: np.ndarray, params: ColorDetectParams) -> np.ndarray:
    """Return a binary mask of pixels matching any of the params' HSV bands."""
    img = bgr
    if params.blur_ksize >= 3 and params.blur_ksize % 2 == 1:
        img = cv2.GaussianBlur(bgr, (params.blur_ksize, params.blur_ksize), 0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask: np.ndarray | None = None
    for band in params.bands:
        band_mask = cv2.inRange(hsv, band.lower(), band.upper())
        mask = band_mask if mask is None else cv2.bitwise_or(mask, band_mask)
    if mask is None:
        return np.zeros(bgr.shape[:2], dtype=np.uint8)

    if params.morph_ksize >= 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (params.morph_ksize, params.morph_ksize)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect_colored_circles(
    bgr: np.ndarray, params: ColorDetectParams, max_results: int = 1
) -> list[CircleDetection]:
    """Detect colored circular blobs, largest first, up to max_results.

    Confidence is synthesized from how well the blob fills its enclosing circle
    (a clean ball fills it almost entirely), so partial/occluded or ragged blobs
    score lower. There is no neural probability here by design.
    """
    mask = build_mask(bgr, params)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    results: list[CircleDetection] = []
    for contour in contours:
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        if radius < params.min_radius_px or radius > params.max_radius_px:
            continue
        circle_area = float(np.pi * radius * radius)
        area = float(cv2.contourArea(contour))
        fill = area / circle_area if circle_area > 0.0 else 0.0
        if fill < params.min_fill_ratio:
            continue
        confidence = float(min(0.99, 0.40 + 0.60 * fill))
        results.append(
            CircleDetection(float(cx), float(cy), float(radius), confidence, float(fill), area)
        )

    results.sort(key=lambda d: d.radius, reverse=True)
    return results[:max_results]
