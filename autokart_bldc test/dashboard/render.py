"""Render the existing perception result for the diagnostic camera view."""
from __future__ import annotations

from math import isfinite
from types import SimpleNamespace

import cv2

from autokart_perception.live_render import rgb_preview


def annotated_rgb(frame, result):
    """Use the shared box renderer, then label candidate depth honestly."""
    image = rgb_preview(frame, SimpleNamespace(perception_result=result),
                        extrinsic_verified=False)
    cones = {cone.id: cone for cone in result.cones}
    label_regions = []
    for detection in result.detections:
        cone = cones.get(detection.id)
        if cone is None or not isfinite(cone.distance_m):
            continue
        x1, y1, x2, y2 = detection.box
        # No '~' here: cv2.FONT_HERSHEY_SIMPLEX renders '~' at this small scale as a short
        # flat squiggle that reads as a minus sign (confirmed by direct render test), making
        # an unverified-but-positive candidate distance look like a negative one. The trailing
        # '?' already conveys "candidate, not a verified measurement" without that ambiguity.
        label = f"{cone.distance_m:.2f}m?"
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .43, 1)
        width, height = text_w + 8, text_h + 7
        max_x, max_y = image.shape[1] - width, image.shape[0] - height
        choices = [(x1, y2 + 3), (x2 - width, y2 + 3),
                   (x1, y1 - height - 3), (x2 + 3, y1)]
        choices = [(max(0, min(x, max_x)), max(0, min(y, max_y))) for x, y in choices]
        def overlap(candidate):
            x, y = candidate
            return sum(max(0, min(x + width, rx + rw) - max(x, rx)) *
                       max(0, min(y + height, ry + rh) - max(y, ry))
                       for rx, ry, rw, rh in label_regions)
        x, y = min(choices, key=overlap)
        label_regions.append((x, y, width, height))
        cv2.rectangle(image, (x, y), (x + width, y + height), (6, 16, 26), -1)
        cv2.putText(image, label, (x + 4, y + text_h + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, (235, 235, 235), 1, cv2.LINE_AA)
    return image
