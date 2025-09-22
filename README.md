# Attitude Control System (ACS) Simulator

This project simulates spacecraft attitude dynamics and control in Python. It’s modular, extensible, and set up for experimentation with controllers, trajectories, and dynamics.

---

## Features

- **Torque-Driven Dynamics:** Rigid-body rotation with Euler’s equations, using `I w_dot + w × (I w) = T` (gyroscopic term included).
- **Quaternion State Propagation:** Scalar-first quaternions with normalization each step; stable small-angle handling.
- **Controllers:** PD and PID controllers with axis-wise torque saturation. Gains can be diagonal or full matrices; off-diagonal coupling via `diag_gain` in `sim.py`.
- **Slew Generation:** Trapezoidal/triangular angular-rate profiles respecting `w_max` and `a_max`, with shortest-path quaternion planning.
- **Visualization:** Plots of body angular velocity and Euler angles vs. time for both the simulation and the generated slew.
- **Clear Conventions:** Nav→body (active) attitude convention throughout; consistent quaternion/Euler conversions.

---

## Important Conventions

- **Quaternion Format:** `[q0, q1, q2, q3]` with `q0` scalar-first. Conjugate is `[q0, -q1, -q2, -q3]`.
- **Multiplication:** `q1.q_prod(q2)` implements the Hamilton product in scalar-first form; composition uses right-multiplication for active rotations.
- **Nav→Body Error:** Controllers use `q_err = q_desired · q_current*`. Shortest-path enforced by flipping the full quaternion if `q_err.q0 < 0`.
- **Feedback Sign (Controllers):** For small angles, `q_err_vec ≈ +δθ/2`. With negative-feedback torque `τ = −Kp e − Kd(w_cur − w_des)`, we set `e = −q_err_vec` so a positive desired angle yields a positive body torque.
- **Euler Angles:** Intrinsic Tait–Bryan sequence `'xyz'` (roll–pitch–yaw). Plots display degrees.
- **Angular Velocity:** Always in the body frame (rad/s).

---

## Project Structure

```
.
├── run.sh                                # Convenience launcher
├── src/
│   └── attitude_control/
│       ├── __init__.py
│       ├── sim.py                        # Simulation setup + loop + plotting
│       ├── command_shaping/
│       │   ├── __init__.py
│       │   ├── slew_generator.py         # Quaternion shortest-path + rate profile
│       │   └── trajectory.py             # Trajectory container + plotting helper
│       ├── controllers/
│       │   ├── __init__.py
│       │   ├── controller.py             # Base class (max torque, interface)
│       │   ├── pd_control.py             # PD controller (negative feedback)
│       │   └── pid_control.py            # PID controller + anti-windup
│       ├── plant/
│       │   ├── __init__.py
│       │   ├── dynamics.py               # Integrators + torque→alpha
│       │   ├── quaternion.py             # Quaternion math + Euler conversions
│       │   ├── spacecraft.py             # Applies torque to state (I, w)
│       │   └── state.py                  # Attitude + rate state container
│       └── utils/
│           ├── __init__.py
│           └── visualization.py          # Euler/omega plots (png)
└── README.md
```

---

## Getting Started

1. **Create and activate a virtual environment (recommended):**
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install -U pip
   ```
2. **Install dependencies:**
   ```bash
   pip install numpy scipy matplotlib
   ```
3. **Run the simulation:**
   ```bash
   # Option A: via helper script
   ./run.sh

   # Option B: as a module
   python3 -m src.attitude_control.sim
   ```

---

## Requirements

- Python 3.10+ (tested with 3.12)
- NumPy
- SciPy
- Matplotlib

---

## Simulation Overview

- **Initial Conditions:** Identity quaternion, zero rates by default (see `sim.py`).
- **Desired Attitude:** Set via Euler angles; example uses roll=60°, pitch=70°, yaw=−40°.
- **Controller:** PID by default with gains in `sim.py` (example: `kp=400`, `ki=3`, `kd=1500`). Max torque set via `max_torque`.
- **Slew Profile:** Generated with `w_max` and `a_max` (example: 5°/s and 1°/s²). Trajectory plots saved separately.
- **Disturbance:** Optional Gaussian torque added in-loop for realism (toggle in `sim.py`).
- **Timestep:** `dt = 0.001 s`; duration `time = 150 s` by default.

Generated plots at the repo root:
- `angular_velocity.png`, `quaternion_euler_angles.png` (simulation results)
- `angular_velocity_slew.png`, `quaternion_euler_angles_slew.png` (commanded slew)

Note: PNGs are `.gitignore`d by default.

---

## Implementation Notes

- **Dynamics Integration:**
  - Angular rates: explicit Euler integration.
  - Attitude: right-multiplication by incremental quaternion `dq`; small-angle case uses `dq ≈ [1, 0.5 w dt]`.
- **Computed-Torque Term (PID):** Adds `w × (I w)` for gyroscopic compensation before saturation.
- **Shortest Path:** If the scalar part of the error quaternion is negative, the quaternion is flipped to follow the minimal rotation.
- **Consistency:** All conversions and multiplications use scalar-first quaternions and the `'xyz'` Euler sequence.

---

## Roadmap / Ideas

- Convert `sim.py` into a configurable class with runtime options.
- Refine disturbance torques and add orbital propagation/major disturbances.
- Actuator realism: rate limits, startup torque, and deadband.
- Alternate frames/coordinate systems and comparison tooling.
- Automated tests and benchmarks for controller performance.

---

Created by Daniel Ludlow, 2025
