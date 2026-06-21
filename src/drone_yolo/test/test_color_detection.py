"""Unit tests for the color detection core (no ROS).

Needs numpy + cv2 (installed with ros-jazzy-cv-bridge / python3-opencv).
Run directly: python3 src/drone_yolo/test/test_color_detection.py
"""

import os
import sys

import numpy as np

try:
    from drone_yolo.color_detection import default_red_params, detect_colored_circles
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from drone_yolo.color_detection import default_red_params, detect_colored_circles  # noqa: E402

import cv2


def _frame_with_red_ball(cx, cy, radius, w=640, h=480):
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:] = (50, 90, 40)  # dull greenish background (low saturation red)
    cv2.circle(frame, (cx, cy), radius, (0, 0, 215), -1)  # BGR saturated red
    return frame


def test_detects_centered_red_ball():
    frame = _frame_with_red_ball(320, 240, 30)
    found = detect_colored_circles(frame, default_red_params())
    assert len(found) == 1, f"expected 1 detection, got {len(found)}"
    d = found[0]
    assert abs(d.cx - 320) <= 3 and abs(d.cy - 240) <= 3, f"center off: {d.cx},{d.cy}"
    assert abs(d.radius - 30) <= 4, f"radius off: {d.radius}"
    assert d.confidence >= 0.7, f"low confidence: {d.confidence}"


def test_no_false_positive_on_plain_background():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (120, 120, 120)  # gray, no saturated red
    found = detect_colored_circles(frame, default_red_params())
    assert found == [], f"expected no detections, got {len(found)}"


def test_picks_largest_of_two():
    frame = _frame_with_red_ball(150, 240, 18)
    cv2.circle(frame, (480, 240), 34, (0, 0, 215), -1)  # bigger ball on the right
    found = detect_colored_circles(frame, default_red_params(), max_results=1)
    assert len(found) == 1
    assert abs(found[0].cx - 480) <= 3, f"should pick the larger ball, got cx={found[0].cx}"


def test_rejects_non_round_blob():
    # A thin red bar is colored but not round; fill ratio should reject it.
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(frame, (300, 235), (520, 245), (0, 0, 215), -1)
    found = detect_colored_circles(frame, default_red_params())
    assert found == [], f"expected non-round blob rejected, got {len(found)}"


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failures = 0
    for test in tests:
        try:
            test()
            print(f"PASS {test.__name__}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"FAIL {test.__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
