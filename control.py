"""
control.py — Ball & Beam control algorithms
============================================

Two classes:

  LeadLag   — first-order lead-lag compensator (continuous-time transfer
              function discretised via the bilinear / Tustin method)

  PID       — proportional-integral-derivative controller with:
                · per-term bookkeeping (last_p, last_i, last_d, last_out)
                · integral enable/disable flag
                · output clamping
                · deadband
"""

import time
import numpy as np


# ── Lead-Lag compensator ──────────────────────────────────────────────────────

class LeadLag:
    """
    Continuous-time transfer function:

        H(s) = (tau_lead * s + 1) / (tau_lag * s + 1)

    Discretised with the bilinear (Tustin) transform at a fixed sample
    period dt = 0.033 s (~30 fps).  Choosing tau_lead > tau_lag gives
    phase lead (speeds up the closed-loop response); tau_lead < tau_lag
    gives phase lag (low-pass smoothing).

    Difference equation (derived from H(s) via s ≈ 2/T * (z-1)/(z+1)):

        y[k] = a * y[k-1] + b0 * x[k] + b1 * x[k-1]

    where the coefficients a, b0, b1 are computed in set().
    """

    def __init__(self, tau_lead=0.5, tau_lag=0.15, dt=0.033):
        self._dt     = dt
        self._y_prev = 0.0
        self._x_prev = 0.0
        self._a = self._b0 = self._b1 = 0.0
        self.set(tau_lead, tau_lag)

    def set(self, tau_lead, tau_lag):
        """Recompute coefficients.  Both taus are clamped to dt/2 for stability."""
        T        = self._dt
        tau_lead = max(tau_lead, T / 2)
        tau_lag  = max(tau_lag,  T / 2)
        alpha    = 2.0 * tau_lead / T
        beta     = 2.0 * tau_lag  / T
        d        = beta + 1.0
        self._a  = (beta  - 1.0) / d
        self._b0 = (alpha + 1.0) / d
        self._b1 = (1.0 - alpha) / d

    def update(self, x):
        """Push one sample through the filter and return the filtered output."""
        y            = self._a * self._y_prev + self._b0 * x + self._b1 * self._x_prev
        self._y_prev = y
        self._x_prev = x
        return y

    def reset(self):
        """Clear filter state (call when re-entering AUTO mode)."""
        self._y_prev = 0.0
        self._x_prev = 0.0


# ── PID controller ────────────────────────────────────────────────────────────

class PID:
    """
    Discrete PID controller — velocity output.

    Control law (when outside deadband):

        P  = Kp * error
        I  = Ki * integral(error * dt)      [only when integral_on]
        D  = Kd * (error - prev_error) / dt

        output = clip(P + I + D, -max_out, +max_out)

    The integral term is clamped to [-1, +1] before multiplication by Ki
    to prevent windup.

    Public attributes after each update():
        last_p, last_i, last_d   — individual term values
        last_out                 — clipped total output
        integral                 — current accumulator value (read-only property)
    """

    def __init__(self, kp, ki, kd, max_out):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.max_out = max_out

        self.integral_on = True

        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t   = None

        self.last_p   = 0.0
        self.last_i   = 0.0
        self.last_d   = 0.0
        self.last_out = 0.0

    @property
    def integral(self):
        return self._integral

    def clear_integral(self):
        """Zero the accumulator (e.g. when toggling the integral term on/off)."""
        self._integral = 0.0

    def reset(self):
        """Full reset — call when switching to/from AUTO mode."""
        self._integral = 0.0
        self._prev_err = 0.0
        self._prev_t   = None
        self.last_p = self.last_i = self.last_d = self.last_out = 0.0

    def update(self, error, deadband=0.0):
        """
        Compute one control step.

        Parameters
        ----------
        error    : float  — signed position error (positive = ball too far right)
        deadband : float  — errors smaller than this return 0 without updating state

        Returns
        -------
        float — clipped PID output
        """
        now = time.time()
        dt  = (now - self._prev_t) if self._prev_t else 0.033
        self._prev_t = now

        if abs(error) < deadband:
            self._prev_err = error
            self.last_p = self.last_i = self.last_d = self.last_out = 0.0
            return 0.0

        if self.integral_on:
            self._integral += error * dt
            self._integral  = np.clip(self._integral, -1.0, 1.0)

        derivative     = (error - self._prev_err) / dt if dt > 0 else 0.0
        self._prev_err = error

        self.last_p   = self.kp * error
        self.last_i   = self.ki * self._integral if self.integral_on else 0.0
        self.last_d   = self.kd * derivative
        self.last_out = float(np.clip(
            self.last_p + self.last_i + self.last_d,
            -self.max_out, self.max_out
        ))
        return self.last_out
