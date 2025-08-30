# Attitude Control System (ACS) Simulator

This project simulates spacecraft attitude dynamics and control using Python. It is modular, extensible, and designed for both experimentation and future scalability.

## Features

- **State Propagation:** Simulates spacecraft attitude and angular velocity using quaternion mathematics.
- **Dynamics:** Models rotational motion based on applied torques and spacecraft inertia, using realistic rigid-body equations.
- **Torque-Based Simulation:** The simulation uses torque as the primary input, reflecting real-world actuator interfaces.
- **Controllers:** Modular controller framework with a PD controller implementation, including torque saturation (max torque limit). Easily extendable for custom control laws.
- **Visualization:** Tools for plotting and visualizing attitude (Euler angles) and angular velocity over time.
- **Quaternion/Euler Conversion:** Accurate conversion between quaternion and Euler angles for analysis and plotting.
- **Tunable Gains:** Controller gains (Kp, Kd) and actuator limits are easily adjustable for tuning and experimentation.

## Folder Structure

```
attitude_control/
├── __init__.py
├── dynamics.py          # Spacecraft rotational dynamics
├── quaternion.py        # Quaternion operations and math utilities
├── sim.py               # Main simulation loop and setup
├── spacecraft.py        # Spacecraft class (inertia, state, torque application)
├── state.py             # State representation (attitude, angular velocity, etc.)
├── visualization.py     # Visualization and plotting tools
├── controllers/         # Modular controllers
│   ├── __init__.py
│   ├── controller.py    # Base controller class (with max torque property)
│   └── PD_control.py    # Proportional-Derivative controller with torque saturation
├── README.md            # Project documentation
```

## Getting Started

1. **Clone the repository** and navigate to the `attitude_control` directory.
2. **Install dependencies:**
   ```bash
   pip install numpy matplotlib
   ```
3. **Run the simulation:**  
   ```bash
   python -m attitude_control.sim
   ```

## Requirements

- Python 3.7+
- NumPy
- Matplotlib

## Notes

- The simulation uses torque as the main input, not angular acceleration.
- The PD controller includes torque saturation to reflect actuator limits.
- Controller gains (Kp, Kd) and max torque are easily tunable in `sim.py`.
- Visualization tools require Matplotlib and plot both angular velocity and Euler angles (in degrees).
- The controller and simulation structure are modular for easy extension and experimentation.

*Created by Daniel Ludlow, 2025*
