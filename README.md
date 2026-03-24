# Ball & Beam — PID Gain Derivation

**Repository:** [smeschke/ball_and_beam](https://github.com/smeschke/ball_and_beam)

A ball rolls on a beam tilted by a stepper motor via a cam mechanism. A webcam detects ball position (normalized 0–1 across the beam length). A PC runs a PID controller and sends signed velocity commands (steps/sec) over serial to an Arduino, which drives the stepper.

Because the output unit is **steps/sec rather than beam angle**, the PID gains are much larger than textbook values. This document derives them from first principles.

![Ball and beam demo](ball_beam.gif)

---

## Physical Parameters

| Parameter | Value |
|---|---|
| Beam active length | 21 in = 0.533 m |
| Stepper resolution | 25,000 steps / revolution |
| Cam full stroke | 12,500 steps → full left to full right |
| Max beam tilt | ~5.5° = 0.096 rad (avg of 6.4° / 4.7°, measured by phone inclinometer) |
| Angle per step | 0.096 / 12,500 = 7.75 × 10⁻⁶ rad/step |
| Ball | Ping Pong |
| MAX_VEL | 8,000 steps/sec |

---

## Plant Model

### Ball dynamics

For a solid sphere rolling without slip on an inclined beam (small angle approximation):

```
pos_ddot  =  (5g/7) × θ / beam_length
           =  (5 × 9.81 / 7) × θ / 0.533
           =  13.15 × θ     [normalized/s²  per  radian]
```

### Plant gain K

The controller commands velocity (steps/sec). Motor position is the integral of velocity, and beam angle is proportional to motor position. This makes the plant a **double integrator** — velocity → position — with effective gain K:

```
K  =  13.15 × (0.096 / 12,500)
   =  13.15 × 7.75 × 10⁻⁶
   =  1.02 × 10⁻⁴   [normalized/s²  per  step/s]
```

> **Sanity check:** at 1,000 steps/sec sustained for 1 second, expected ball displacement ≈ ½ × K × v × t² = ½ × 1.02×10⁻⁴ × 1000 × 1 = 0.051, or about 5% of beam length. Physically reasonable.

---

## PID Gain Derivation

With a PD controller on a double integrator G(s) = K/s², the closed-loop characteristic equation is:

```
s²  +  K·Kd·s  +  K·Kp  =  0
```

Matching to the standard second-order form `s² + 2ζωn·s + ωn²` gives:

```
Kp  =  ωn²    / K
Kd  =  2·ζ·ωn / K
```

For critical damping (ζ = 1) the settling time is `t_s ≈ 4/ωn`, so `ωn = 4 / t_settle`.

| Desired settling time | ωn (rad/s) | Kp | Kd |
|---|---|---|---|
| 1.5 s (fast) | 2.67 | 69,900 | 52,400 |
| 2.0 s (recommended) | 2.00 | 39,200 | 39,200 |
| 3.0 s (conservative) | 1.33 | 17,400 | 26,100 |

> **Note:** in practice the system works better slightly underdamped (ζ ≈ 0.6), allowing one or two oscillations before settling. This is due to real-world friction and motor deadband near the setpoint. Use ζ = 0.6 in the Kd formula to account for this.

---

## Lead-Lag Compensator — Critical Role

The system uses a first-order lead-lag filter on the PID output:

```
H(s)  =  (τ_lead · s + 1) / (τ_lag · s + 1)
```

**Current tuning:** τ_lead = 500 ms, τ_lag = 150 ms.

Since τ_lead > τ_lag this is a **phase-lead compensator** — it amplifies mid-to-high frequencies and adds phase advance, acting as a strong additional derivative term on top of the PID's Kd.

This is why the experimentally working gains (Kp = 1000, Kd = 5830) are roughly 30–40× lower than the theoretical values above. **The lead-lag is supplying most of the derivative action.** The PID gains and the lead-lag must be tuned together — changing one without the other will destabilize the system.

> Disabling the lead-lag (press `L`) causes the system to oscillate and run away even with Kd = 5830. This is expected — the raw PD gains are insufficient without the compensator.

| Condition | Effect |
|---|---|
| τ_lead > τ_lag | Phase lead — faster response, more derivative action, can increase overshoot |
| τ_lead < τ_lag | Phase lag — smoothing, reduces noise sensitivity |
| τ_lead = τ_lag | Unity — filter has no effect |

To reduce overshoot from disturbances: lower τ_lead toward τ_lag. To reduce noise-driven chatter in the d_term: increase τ_lag.

---

## Keyboard Controls

| Key | Action |
|---|---|
| `A` | Toggle AUTO / MANUAL mode |
| `I` | Toggle integral term on/off |
| `L` | Toggle lead-lag filter on/off |
| `S` / `D` | Manual CCW / CW (manual mode only) |
| `Q` / `Esc` | Quit |
