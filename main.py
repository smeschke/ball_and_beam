#!/usr/bin/env python3
"""
Ball & Beam — main loop
  A       : toggle AUTO mode
  I       : toggle integral term
  L       : toggle lead-lag filter
  S / D   : manual CCW / CW (only in manual mode)
  Q / Esc : quit
"""

import cv2
import time
from collections import deque

from config    import (CAM_INDEX, KP, KI, KD, MAX_VEL, MANUAL_VEL,
                       GRAPH_W, GRAPH_H, GRAPH_HISTORY, WIN, WIN_GRAPH, WIN_ROI, WIN_TABLE)
from control   import PID, LeadLag
from detection import detect_ball
from serial_io import find_arduino, open_serial, send_velocity
from recorder  import Recorder
from ui        import (make_trackbars, read_pid_sliders, read_detection_sliders,
                       update_display)


def main():
    # ── setup ─────────────────────────────────────────────────────────────────
    ser = open_serial(find_arduino())

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")
    ret, frame0 = cap.read()
    if not ret:
        raise RuntimeError("Cannot read from camera")
    fh, fw      = frame0.shape[:2]
    half_w      = GRAPH_W // 2
    cam_h_scaled = int(fh * half_w / fw)

    rec = Recorder(cam_h_scaled)

    cv2.namedWindow(WIN,       cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN,       860,     640)
    cv2.namedWindow(WIN_ROI,   cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN_ROI,   640,     200)
    cv2.namedWindow(WIN_GRAPH, cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN_GRAPH, GRAPH_W, GRAPH_H)
    cv2.namedWindow(WIN_TABLE, cv2.WINDOW_NORMAL); cv2.resizeWindow(WIN_TABLE, 700,     700)
    make_trackbars()

    pid         = PID(KP, KI, KD, MAX_VEL)
    leadlag     = LeadLag(tau_lead=0.5, tau_lag=0.15)
    auto_mode   = False
    integral_on = True
    leadlag_on  = True
    hist        = deque(maxlen=GRAPH_HISTORY)
    t0          = time.time()
    motor_pos   = 0.0
    prev_t      = t0

    # ── loop ──────────────────────────────────────────────────────────────────
    while True:
        ret, frame = cap.read()
        if not ret:
            print("frame lost"); break

        h, w = frame.shape[:2]
        key  = cv2.waitKey(1) & 0xFF

        kp, ki, kd, deadband, setpoint, tau_lead, tau_lag = read_pid_sliders()
        v_min, s_max, min_area, y0, y1, x0, x1           = read_detection_sliders(h, w)

        bx, by, radius, ball_x_norm = detect_ball(frame, v_min, s_max, min_area,
                                                   y0, y1, x0, x1)

        # ── control ───────────────────────────────────────────────────────────
        pid.kp, pid.ki, pid.kd = kp, ki, kd
        pid.integral_on        = integral_on
        leadlag.set(tau_lead, tau_lag)

        error = (ball_x_norm - setpoint) if ball_x_norm is not None else 0.0
        vel   = -pid.update(error, deadband)
        if leadlag_on:
            vel = leadlag.update(vel)

        send_velocity(ser, vel) if auto_mode else _manual(ser, key)
        # ─────────────────────────────────────────────────────────────────────

        t_now      = time.time() - t0
        dt         = time.time() - prev_t
        prev_t     = time.time()
        motor_pos += vel * dt if auto_mode else 0.0

        pos_val = ball_x_norm if ball_x_norm is not None else float("nan")
        hist.append({"pos": pos_val, "setpoint": setpoint,
                     "err": error,   "vel": vel, "motor_pos": motor_pos})

        rec.write(t_now, pos_val, setpoint, error, pid, vel, motor_pos,
                  kp, ki, kd, deadband, tau_lead, tau_lag,
                  bx, by, radius, auto_mode, integral_on, leadlag_on)

        table_state = {
            "auto": auto_mode, "pos": pos_val,    "sp":  setpoint,
            "err":  error,     "kp":  kp,          "ki":  ki,   "kd": kd,
            "p":    pid.last_p,"i":   pid.last_i,  "d":   pid.last_d,
            "pid_out":   pid.last_out,  "vel":      vel,
            "motor_pos": motor_pos,     "integral": pid.integral,
            "deadband":  deadband,      "tau_lead": tau_lead,
            "tau_lag":   tau_lag,       "i_on":     integral_on,
            "ll_on":     leadlag_on,
        }
        combined = update_display(frame, bx, by, radius, ball_x_norm, error,
                                  setpoint, deadband, x0, y0, x1, y1,
                                  auto_mode, integral_on, leadlag_on, ser,
                                  hist, table_state, half_w, cam_h_scaled)
        rec.write_frame(combined)

        # ── keys ──────────────────────────────────────────────────────────────
        if key in (ord('q'), 27):
            break
        elif key == ord('a'):
            auto_mode = not auto_mode
            pid.reset(); leadlag.reset(); send_velocity(ser, 0)
            print(f"[mode] {'AUTO' if auto_mode else 'MANUAL'}")
        elif key == ord('i'):
            integral_on = not integral_on
            pid.clear_integral()
            print(f"[integral] {'ON' if integral_on else 'OFF'}")
        elif key == ord('l'):
            leadlag_on = not leadlag_on
            leadlag.reset()
            print(f"[leadlag] {'ON' if leadlag_on else 'OFF'}")

    # ── cleanup ───────────────────────────────────────────────────────────────
    send_velocity(ser, 0)
    cap.release()
    rec.close()
    if ser:
        ser.close()
    cv2.destroyAllWindows()


def _manual(ser, key):
    """Send manual velocity while not in AUTO mode."""
    if   key == ord('d'): send_velocity(ser,  MANUAL_VEL)
    elif key == ord('s'): send_velocity(ser, -MANUAL_VEL)
    else:                 send_velocity(ser,  0)


if __name__ == "__main__":
    main()
