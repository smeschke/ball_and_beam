"""
recorder.py — CSV and video recording
"""

import csv
import cv2
import numpy as np
import os
from datetime import datetime

from config import DESKTOP, GRAPH_W, GRAPH_H


class Recorder:
    """Opens a CSV file and an AVI VideoWriter; write() logs one sample per frame."""

    CSV_HEADER = [
        "time_s", "pos", "setpoint", "error",
        "p_term", "i_term", "d_term", "pid_out",
        "velocity", "motor_pos", "kp", "ki", "kd",
        "deadband", "lead_ms", "lag_ms", "integral_accum",
        "ball_px_x", "ball_px_y", "ball_radius",
        "auto", "integral_on", "leadlag_on",
    ]

    def __init__(self, cam_h_scaled):
        ts            = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(DESKTOP, f"ball_beam_{ts}.csv")
        self.vid_path = os.path.join(DESKTOP, f"ball_beam_{ts}.avi")

        self._csv_file   = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(self.CSV_HEADER)

        fourcc       = cv2.VideoWriter_fourcc(*"XVID")
        combined_h   = cam_h_scaled + GRAPH_H
        self._vwriter = cv2.VideoWriter(
            self.vid_path, fourcc, 30.0, (GRAPH_W, combined_h)
        )

        print(f"[record] video → {self.vid_path}")
        print(f"[record] csv   → {self.csv_path}")

    def write(self, t_now, pos_val, setpoint, error, pid, vel, motor_pos,
              kp, ki, kd, deadband, tau_lead, tau_lag,
              bx, by, radius, auto_mode, integral_on, leadlag_on):
        self._csv_writer.writerow([
            f"{t_now:.4f}",          f"{pos_val:.4f}",       f"{setpoint:.4f}",
            f"{error:.4f}",          f"{pid.last_p:.2f}",    f"{pid.last_i:.2f}",
            f"{pid.last_d:.2f}",     f"{pid.last_out:.1f}",  f"{vel:.1f}",
            f"{motor_pos:.1f}",      f"{kp:.1f}",            f"{ki:.1f}",
            f"{kd:.1f}",             f"{deadband:.4f}",      f"{tau_lead*1000:.0f}",
            f"{tau_lag*1000:.0f}",   f"{pid.integral:.6f}",
            bx if bx is not None else "",
            by if by is not None else "",
            radius if radius is not None else "",
            int(auto_mode), int(integral_on), int(leadlag_on),
        ])

    def write_frame(self, combined_frame):
        self._vwriter.write(combined_frame)

    def close(self):
        self._csv_file.close()
        self._vwriter.release()
        print(f"[done] saved {self.csv_path}")
        print(f"[done] saved {self.vid_path}")
