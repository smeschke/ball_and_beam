"""
detection.py — Ball detection via HSV colour thresholding
"""

import cv2

from config import WIN_ROI


def detect_ball(frame, v_min, s_max, min_area, y0, y1, x0, x1):
    """
    Locate the ball in *frame* within the ROI [y0:y1, x0:x1].

    The ball is assumed to be a bright, low-saturation object (white/silver).
    The HSV mask keeps pixels where:
        V (value/brightness) >= v_min
        S (saturation)       <= s_max

    Returns
    -------
    (bx, by, radius, ball_x_norm)
        bx, by       — ball centre in full-frame pixel coordinates
        radius       — enclosing-circle radius in pixels
        ball_x_norm  — horizontal position normalised to [0, 1] within the ROI
    All four are None if no ball is found.
    """
    roi = frame[y0:y1, x0:x1]
    cv2.imshow(WIN_ROI, roi)

    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, v_min), (180, s_max, 255))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, None, None

    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None, None, None, None

    (bx, by), radius = cv2.minEnclosingCircle(c)
    ball_x_norm = bx / (x1 - x0)
    return int(bx) + x0, int(by) + y0, int(radius), float(ball_x_norm)
