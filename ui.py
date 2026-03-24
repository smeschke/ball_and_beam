"""
ui.py — OpenCV windows, trackbars, and rendering
"""

import cv2
import numpy as np

from config import (
    WIN, WIN_TUNE, WIN_ROI, WIN_PID, WIN_GRAPH, WIN_TABLE,
    GRAPH_W, GRAPH_H, GRAPH_HISTORY, MAX_VEL,
    V_MIN_INIT, S_MAX_INIT, AREA_INIT,
    ROI_TOP_INIT, ROI_BOT_INIT, ROI_LEFT_INIT, ROI_RIGHT_INIT,
)


# ── trackbars ─────────────────────────────────────────────────────────────────

def make_trackbars():
    cv2.namedWindow(WIN_PID, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_PID, 420, 400)
    cv2.createTrackbar("Kp x100",       WIN_PID,  4,    200,  lambda _: None)
    cv2.createTrackbar("Ki x10",        WIN_PID,  2,    200,  lambda _: None)
    cv2.createTrackbar("Kd x10",        WIN_PID,  500, 2000,  lambda _: None)
    cv2.createTrackbar("Deadband x001", WIN_PID,  1,   100,   lambda _: None)
    cv2.createTrackbar("Setpoint %",    WIN_PID,  50,  100,   lambda _: None)
    cv2.createTrackbar("Lead ms",       WIN_PID,  500, 5000,  lambda _: None)
    cv2.createTrackbar("Lag  ms",       WIN_PID,  150, 5000,  lambda _: None)

    cv2.namedWindow(WIN_TUNE, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_TUNE, 420, 320)
    cv2.createTrackbar("V min",     WIN_TUNE, V_MIN_INIT,     255, lambda _: None)
    cv2.createTrackbar("S max",     WIN_TUNE, S_MAX_INIT,     255, lambda _: None)
    cv2.createTrackbar("Area/10",   WIN_TUNE, AREA_INIT,      500, lambda _: None)
    cv2.createTrackbar("ROI top %", WIN_TUNE, ROI_TOP_INIT,    99, lambda _: None)
    cv2.createTrackbar("ROI bot %", WIN_TUNE, ROI_BOT_INIT,   100, lambda _: None)
    cv2.createTrackbar("ROI lft %", WIN_TUNE, ROI_LEFT_INIT,   99, lambda _: None)
    cv2.createTrackbar("ROI rgt %", WIN_TUNE, ROI_RIGHT_INIT, 100, lambda _: None)


def read_pid_sliders():
    kp       = cv2.getTrackbarPos("Kp x100",       WIN_PID) * 100.0
    ki       = cv2.getTrackbarPos("Ki x10",        WIN_PID) * 10.0
    kd       = cv2.getTrackbarPos("Kd x10",        WIN_PID) * 10.0
    deadband = cv2.getTrackbarPos("Deadband x001", WIN_PID) * 0.001
    setpoint = cv2.getTrackbarPos("Setpoint %",    WIN_PID) / 100.0
    tau_lead = cv2.getTrackbarPos("Lead ms",       WIN_PID) * 0.001
    tau_lag  = cv2.getTrackbarPos("Lag  ms",       WIN_PID) * 0.001
    return kp, ki, kd, deadband, setpoint, tau_lead, tau_lag


def read_detection_sliders(h, w):
    v_min    = cv2.getTrackbarPos("V min",     WIN_TUNE)
    s_max    = cv2.getTrackbarPos("S max",     WIN_TUNE)
    min_area = cv2.getTrackbarPos("Area/10",   WIN_TUNE) * 10

    y0 = int(cv2.getTrackbarPos("ROI top %", WIN_TUNE) / 100 * h)
    y1 = max(int(cv2.getTrackbarPos("ROI bot %", WIN_TUNE) / 100 * h), y0 + 1)
    x0 = int(cv2.getTrackbarPos("ROI lft %", WIN_TUNE) / 100 * w)
    x1 = max(int(cv2.getTrackbarPos("ROI rgt %", WIN_TUNE) / 100 * w), x0 + 1)

    return v_min, s_max, max(min_area, 1), y0, y1, x0, x1


# ── computation table ─────────────────────────────────────────────────────────

def draw_table(w, h, state):
    """
    Render the live PID computation breakdown as an image (w x h pixels).

    state dict keys:
        auto, pos, sp, err, kp, ki, kd,
        p, i, d, pid_out, vel, motor_pos,
        integral, deadband, tau_lead, tau_lag,
        i_on, ll_on
    """
    img     = np.zeros((h, w, 3), dtype=np.uint8)
    ON_COL  = (0, 220, 80)
    OFF_COL = (0, 80, 220)

    i_col   = ON_COL  if state['i_on']  else OFF_COL
    ll_col  = ON_COL  if state['ll_on'] else OFF_COL
    i_tag   = "ON"    if state['i_on']  else "OFF"
    ll_tag  = "ON"    if state['ll_on'] else "OFF"
    pos_str = "---"   if np.isnan(state['pos']) else f"{state['pos']:.4f}"
    err_str = "---"   if np.isnan(state['pos']) else f"{state['err']:+.4f}"

    rows = [
        ("MODE",                 "AUTO" if state['auto'] else "MANUAL",
                                 (0, 180, 255) if state['auto'] else (100, 255, 100)),
        (None, None, None),
        ("Position",             pos_str,                                   (100, 255, 100)),
        ("Setpoint",             f"{state['sp']:.4f}",                      (0, 220, 220)),
        ("Error  (pos - sp)",    err_str,                                   (220, 100, 100)),
        (None, None, None),
        ("Kp",                   f"{state['kp']:.1f}",                      (150, 150, 255)),
        ("Ki",                   f"{state['ki']:.1f}   [{i_tag}]",          i_col),
        ("Kd",                   f"{state['kd']:.1f}",                      (255, 200, 100)),
        (None, None, None),
        ("P  = Kp x error",      f"{state['p']:+.1f}",                      (150, 150, 255)),
        ("I  = Ki x integral",   f"{state['i']:+.1f}",                      i_col),
        ("D  = Kd x deriv",      f"{state['d']:+.1f}",                      (255, 200, 100)),
        (None, None, None),
        ("PID out (P+I+D clipped)", f"{state['pid_out']:+.1f}",             (200, 200, 200)),
        ("Lead-Lag",             f"[{ll_tag}]  lead={state['tau_lead']*1000:.0f}ms"
                                 f"  lag={state['tau_lag']*1000:.0f}ms",    ll_col),
        ("Final velocity",       f"{state['vel']:+.1f}  steps/s",           (255, 150, 50)),
        (None, None, None),
        ("Integral accum",       f"{state['integral']:.5f}",                i_col),
        ("Motor position",       f"{state['motor_pos']:.1f}",               (200, 100, 255)),
        ("Deadband",             f"+/-{state['deadband']:.3f}",             (80, 80, 80)),
    ]

    pad_x   = max(int(w * 0.03), 6)
    val_x   = int(w * 0.55)
    title_h = max(int(h * 0.06), 20)
    row_h   = max((h - title_h) // len(rows), 1)
    fs      = max(row_h * 0.022, 0.3)
    thick   = max(1, int(fs * 1.8))

    cv2.rectangle(img, (0, 0), (w, title_h), (25, 25, 25), -1)
    cv2.putText(img, "PID Computation",
                (pad_x, int(title_h * 0.75)),
                cv2.FONT_HERSHEY_SIMPLEX, fs * 1.1, (180, 180, 180), thick)

    for idx, (label, value, col) in enumerate(rows):
        y_base = title_h + idx * row_h + int(row_h * 0.75)
        if label is None:
            sep_y = title_h + idx * row_h + row_h // 2
            cv2.line(img, (pad_x, sep_y), (w - pad_x, sep_y), (35, 35, 35), 1)
        else:
            cv2.putText(img, label + ":",
                        (pad_x, y_base), cv2.FONT_HERSHEY_SIMPLEX, fs, (130, 130, 130), 1)
            cv2.putText(img, str(value),
                        (val_x, y_base), cv2.FONT_HERSHEY_SIMPLEX, fs, col, thick)

    cv2.line(img, (w - 1, 0), (w - 1, h), (55, 55, 55), 2)
    return img


# ── graph panel ───────────────────────────────────────────────────────────────

def draw_graphs(hist):
    """Render the scrolling time-series graph for position, velocity, and motor position."""
    GW, GH = GRAPH_W, GRAPH_H
    img = np.zeros((GH, GW, 3), dtype=np.uint8)

    motor_vals = [d["motor_pos"] for d in hist if "motor_pos" in d] or [0.0]
    mp_min = min(motor_vals); mp_max = max(motor_vals)
    mp_pad = max((mp_max - mp_min) * 0.1, 50)
    mp_min -= mp_pad; mp_max += mp_pad

    panels = [
        ("Position",  "pos",       (100, 255, 100),  0.0,     1.0),
        ("Velocity",  "vel",       (255, 150,  50), -MAX_VEL, MAX_VEL),
        ("Motor Pos", "motor_pos", (200, 100, 255),  mp_min,  mp_max),
    ]

    N       = len(hist)
    n_pan   = len(panels)
    lmargin = 55
    pad     = 8
    title_h = 22
    pan_h   = (GH - title_h - pad * (n_pan + 1)) // n_pan

    cv2.putText(img, "Ball & Beam — live graphs",
                (lmargin, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    for i, (label, key, col, y_min, y_max) in enumerate(panels):
        py = title_h + pad + i * (pan_h + pad)
        pw = GW - lmargin - pad

        cv2.rectangle(img, (lmargin, py), (lmargin + pw, py + pan_h), (18, 18, 18), -1)
        cv2.rectangle(img, (lmargin, py), (lmargin + pw, py + pan_h), (55, 55, 55),  1)

        def to_px(val):
            return py + pan_h - int(np.clip((val - y_min) / (y_max - y_min), 0, 1) * pan_h)

        if y_min < 0 < y_max:
            cv2.line(img, (lmargin, to_px(0)), (lmargin + pw, to_px(0)), (55, 55, 55), 1)

        if N > 1:
            pts = []
            for j, d in enumerate(hist):
                x = lmargin + int(j * pw / GRAPH_HISTORY)
                y = to_px(d.get(key, 0.0))
                pts.append((x, y))
            cv2.polylines(img, [np.array(pts, dtype=np.int32)],
                          False, col, 1, cv2.LINE_AA)

        if key == "pos" and N > 1:
            sp_pts = []
            for j, d in enumerate(hist):
                x = lmargin + int(j * pw / GRAPH_HISTORY)
                y = to_px(d.get("setpoint", 0.5))
                sp_pts.append((x, y))
            cv2.polylines(img, [np.array(sp_pts, dtype=np.int32)],
                          False, (0, 220, 220), 1, cv2.LINE_AA)

        cv2.putText(img, label,
                    (4, py + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.38, col, 1)
        cv2.putText(img, f"{y_max:.0f}",
                    (4, py + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
        cv2.putText(img, f"{y_min:.0f}",
                    (4, py + pan_h), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)

    return img


# ── camera overlay ────────────────────────────────────────────────────────────

def draw_overlay(frame, bx, by, radius, ball_x_norm, error, setpoint,
                 deadband, x0, y0, x1, y1, auto_mode, integral_on, leadlag_on, ser):
    """Draw all annotations directly onto *frame* (in-place)."""
    h, w = frame.shape[:2]

    # ROI box and setpoint line
    cv2.rectangle(frame, (x0, y0), (x1, y1), (80, 80, 80), 1)
    sp_px = x0 + int(setpoint * (x1 - x0))
    cv2.line(frame, (sp_px, y0), (sp_px, y1), (0, 255, 255), 1)

    if bx is not None:
        cv2.circle(frame, (bx, by), radius, (0, 255, 0), 2)
        cv2.circle(frame, (bx, by), 3,      (0, 255, 0), -1)
        cv2.putText(frame, f"pos={ball_x_norm:.2f}  err={error:+.2f}  db={deadband:.3f}",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    else:
        cv2.putText(frame, "NOT DETECTED",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 80, 255), 1)

    mode_str = "AUTO" if auto_mode else "MANUAL"
    mode_col = (0, 180, 255) if auto_mode else (100, 255, 100)
    cv2.putText(frame, mode_str, (w - 110, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_col, 2)

    ok_txt = "Arduino OK" if ser else "No Arduino"
    ok_col = (0, 220, 80)  if ser else (0, 80, 220)
    cv2.putText(frame, ok_txt, (w - 140, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ok_col, 1)

    int_col = (0, 220, 80) if integral_on else (0, 80, 220)
    cv2.putText(frame, "I: ON" if integral_on else "I: OFF",
                (w - 140, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, int_col, 1)

    ll_col = (0, 220, 80) if leadlag_on else (0, 80, 220)
    cv2.putText(frame, "LL: ON" if leadlag_on else "LL: OFF",
                (w - 140, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ll_col, 1)

    cv2.putText(frame, "A:auto  I:integral  L:leadlag  S:CCW  D:CW  Q:quit",
                (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)


# ── combined display + video frame ────────────────────────────────────────────

def update_display(frame, bx, by, radius, ball_x_norm, error, setpoint,
                   deadband, x0, y0, x1, y1, auto_mode, integral_on, leadlag_on,
                   ser, hist, table_state, half_w, cam_h_scaled):
    """
    Annotate the camera frame, render the table and graphs, show all windows,
    and return the combined frame ready to write to video.
    """
    draw_overlay(frame, bx, by, radius, ball_x_norm, error, setpoint,
                 deadband, x0, y0, x1, y1, auto_mode, integral_on, leadlag_on, ser)
    cv2.imshow(WIN, frame)

    table_img = draw_table(700, 700, table_state)
    cv2.imshow(WIN_TABLE, table_img)

    graph = draw_graphs(hist)
    cv2.imshow(WIN_GRAPH, graph)

    cam_half   = cv2.resize(frame, (half_w, cam_h_scaled))
    table_half = cv2.resize(draw_table(half_w, cam_h_scaled, table_state),
                            (half_w, cam_h_scaled))
    return np.vstack([np.hstack([cam_half, table_half]), graph])
