# Attitude Control System (ACS) Simulator

This project simulates spacecraft attitude dynamics and control in Python. It’s modular, extensible, and set up for experimentation with controllers, trajectories, and dynamics.

---

## Features

- **Torque-Driven Dynamics:** Rigid-body rotation with Euler’s equations, using `I w_dot + w × (I w) = T` (gyroscopic term included).
- **Quaternion State Propagation:** Scalar-first quaternions with normalization each step; stable small-angle handling.
- **Controllers:** PD and PID controllers (anti-windup + saturation). Gains currently diagonal; easily extended.
- **Multi-Waypoint Slews:** Trapezoidal / triangular angular-rate profiles respecting `w_max`, `a_max`, shortest-path quaternion planning between successive goals.
- **Trajectory Stitching:** Automatic hold insertion so each segment arrives at its specified absolute time.
- **Visualization:** Plots of body angular velocity and Euler angles vs. time for each spacecraft.
- **Clear Conventions:** Nav→Body (active) convention; consistent quaternion/Euler conversions.

---

## Important Conventions

- **Quaternion Format:** `[q0, q1, q2, q3]` with `q0` scalar-first. Conjugate is `[q0, -q1, -q2, -q3]`.
- **Multiplication:** `q1.q_prod(q2)` implements Hamilton product (scalar-first); right-multiplication for active composition.
- **Nav→Body Error:** `q_err = q_desired * q_current*`; if `q_err.q0 < 0`, flip sign (shortest path).
- **Feedback Sign:** Small-angle: `q_err_vec ≈ +δθ/2`; controller uses `e = -q_err_vec` so positive desired angle → positive body torque.
- **Euler Angles:** Intrinsic Tait–Bryan `'xyz'` (roll–pitch–yaw), plotted in degrees.
- **Angular Velocity:** Body frame (rad/s).

---

## Project Structure

```
.
├── run.sh                                # Convenience launcher
├── src/
│   └── attitude_control/
│       ├── __init__.py
│       ├── main.py                       # Example setup: spacecraft, goals, run Simulation
│       ├── simulation.py                 # Simulation class (planning + loop + plotting hook)
│       ├── command_shaping/
│       │   ├── __init__.py
│       │   ├── slew_generator.py         # Quaternion shortest-path + rate profile
│       │   └── trajectory.py             # Trajectory container + merge logic
│       ├── controllers/
│       │   ├── __init__.py
│       │   ├── controller.py             # Base class (interface + saturation)
│       │   ├── pd_control.py             # PD controller
│       │   └── pid_control.py            # PID controller + anti-windup
│       ├── plant/
│       │   ├── __init__.py
│       │   ├── dynamics.py               # Torque → angular accel (includes gyro term)
│       │   ├── quaternion.py             # Quaternion math + Euler conversions
│       │   ├── spacecraft.py             # Spacecraft class (state + stepping)
│       │   └── state.py                  # Attitude + rate state container
│       └── utils/
│           ├── __init__.py
│           └── visualization.py          # Euler/omega plots (png)
└── README.md
```

---

## Getting Started

1. **Create & activate a virtual environment (recommended):**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install -U pip
   ```
2. **Install dependencies:**
   ```bash
   pip install numpy scipy matplotlib
   ```
3. **Run the simulation example:**
   ```bash
   python3 -m src.attitude_control.main
   # or
   ./run.sh
   ```

---

## Requirements

- Python 3.10+ (developed on 3.12)
- NumPy
- SciPy
- Matplotlib

---

## Simulation Overview

- **Initial Conditions:** Identity quaternion, zero rates (`main.py`).
- **Waypoints:** Three target attitudes (Euler → quaternion) with specified arrival times; final state held afterward.
- **Controller:** PID (default gains: 400 / 3 / 1500, max torque 100 N·m) per axis.
- **Slew Planning:** Uses `w_max = 0.1 rad/s`, `a_max = 0.05 rad/s²` to build per-segment profiles; inserts hold segments automatically.
- **Disturbance:** Small Gaussian torque added each step (illustrative).
- **Timestep / Duration:** `dt = 0.01 s`, total `time = 200 s` (adjust in `main.py`).
- **Outputs:** Plots (e.g. `craft_1_angular_velocity.png`, `craft_1_quaternion_euler_angles.png`) saved at repo root.

---

## Implementation Notes

- **Dynamics Integration:** Explicit Euler for `w`; quaternion updated via incremental rotation (`dq ≈ [1, 0.5 w dt]`).
- **Gyroscopic Term:** `w × (I w)` included when mapping torque to angular acceleration.
- **Shortest Path:** Quaternion sign flipped if scalar part negative.
- **Trajectory Merge:** Feasibility check ensures arrival times are consistent before concatenation.
- **State History:** Stored as object array of `State` for post-run analysis & plotting.

---

## Roadmap / Ideas

- Configurable CLI / YAML input instead of hard-coded `main.py`.
- More realistic actuators (rate limits, deadband, startup transients).
- Disturbance & orbital environment models.
- Additional controllers (LQR, adaptive, feedforward) + automated tests.
- Profiling & performance benchmarking harness.

---

Created by Daniel Ludlow, 2025
