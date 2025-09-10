# Attitude Control System (ACS) Simulator

This project simulates spacecraft attitude dynamics and control using Python. It is modular, extensible, and designed for both experimentation and future scalability.

---

## Features

- **State Propagation:** Simulates spacecraft attitude and angular velocity using quaternion mathematics.
- **Dynamics:** Models rotational motion based on applied torques and spacecraft inertia, using realistic rigid-body equations.
- **Torque-Based Simulation:** The simulation uses torque as the primary input, reflecting real-world actuator interfaces.
- **Controllers:** Modular controller framework with PD and PID controller implementations, including torque saturation (max torque limit). Easily extendable for custom control laws.
- **Visualization:** Tools for plotting and visualizing attitude (Euler angles) and angular velocity over time.
- **Quaternion/Euler Conversion:** Accurate conversion between quaternion and Euler angles for analysis and plotting.
- **Tunable Gains:** Controller gains (Kp, Kd, Ki) and actuator limits are easily adjustable for tuning and experimentation.

---

## Important Conventions

- **Quaternion Format:** `[q0, q1, q2, q3]` where `q0` is the scalar (real) part, and `[q1, q2, q3]` are the vector (imaginary) parts.
- **Quaternion Multiplication:** `q1.q_prod(q2)` computes `q1 * q2` using the scalar-first convention.
- **Quaternion Normalization:** Quaternions are normalized after each integration step to prevent drift.
- **Quaternion to Euler Angles:**
  - The conversion uses the `'xyz'` (roll-pitch-yaw) intrinsic Tait-Bryan angle sequence.
  - When plotting, Euler angles are converted to degrees.
- **Euler Angles to Quaternion:**
  - Use `Quaternion.from_euler_angles(roll, pitch, yaw)` (angles in radians).
- **Inertia Tensor:**
  - The inertia tensor `I` is a 3x3 numpy array, with units of kg·m².
  - Example values are provided for a 1U CubeSat.
- **Angular Velocity:** Always in the body frame, in radians per second.
- **Torque Saturation:** Controller outputs are clipped axis-wise to the specified `max_torque` value.
- **Simulation Time Step:** Default is `dt = 0.001` seconds for high-fidelity integration.

---

## Project Structure

```
src/
  attitude_control/
    __init__.py
    sim.py                # Main simulation loop and setup
    spacecraft.py         # Spacecraft class (inertia, state, torque application)
    controllers/
      __init__.py
      controller.py       # Base Controller class (with max torque property)
      pd_control.py       # Proportional-Derivative controller with torque saturation
      pid_control.py      # Proportional-Integral-Derivative controller (optional)
    object_state/
      __init__.py
      dynamics.py         # Spacecraft rotational dynamics
      quaternion.py       # Quaternion operations and math utilities
      state.py            # State representation (attitude, angular velocity, etc.)
    utils/
      __init__.py
      visualization.py    # Visualization and plotting tools
```

---

## Getting Started

1. **Clone the repository** and navigate to the project root directory.
2. **Install dependencies:**
   ```bash
   pip install numpy matplotlib scipy
   ```
3. **Run the simulation:**
   ```bash
   cd /path/to/attitude_control
   python3 -m src.attitude_control.sim
   ```

---

## Requirements

- Python 3.7+
- NumPy
- Matplotlib
- SciPy

---

## Notes

- The simulation uses torque as the main input, not angular acceleration.
- The PD and PID controllers include torque saturation to reflect actuator limits.
- Controller gains (Kp, Kd, Ki) and max torque are easily tunable in `sim.py`.
- Visualization tools require Matplotlib and plot both angular velocity and Euler angles (in degrees).

---
*Created by Daniel Ludlow, 2025*
