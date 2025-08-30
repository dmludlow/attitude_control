# Attitude Determination and Control System (ADCS) Simulator

This project simulates spacecraft attitude dynamics and control using Python. It is modular, extensible, and designed for both experimentation and future scalability.

## Features

- **State Propagation:** Simulates spacecraft attitude and angular velocity using quaternion mathematics.
- **Dynamics:** Models rotational motion based on applied torques and spacecraft inertia, using realistic rigid-body equations.
- **Torque-Based Simulation:** The simulation now accepts torque as the primary input, reflecting real-world actuator interfaces.
- **Controllers:** Modular controller framework with a PD controller implementation. Easily extendable for custom control laws.
- **Visualization:** Tools for plotting and visualizing attitude and angular velocity over time.

## Folder Structure

```
attitude_control/
├── __init__.py
├── control.py           # (Legacy/stub) Control law implementations
├── dynamics.py          # Spacecraft rotational dynamics
├── quaternion.py        # Quaternion operations and math utilities
├── sim.py               # Main simulation loop and setup
├── spacecraft.py        # Spacecraft class (inertia, state, torque application)
├── state.py             # State representation (attitude, angular velocity, etc.)
├── visualization.py     # Visualization and plotting tools
├── controllers/         # Modular controllers
│   ├── __init__.py
│   ├── controller.py    # Base controller class
│   └── PD_control.py    # Proportional-Derivative controller implementation
├── README.md            # Project documentation
```

## Getting Started

1. **Clone the repository** and navigate to the `attitude_control` directory.
2. **Run the simulation:**  
   ```bash
   python sim.py
   ```

## Requirements

- Python 3.7+
- NumPy
- Matplotlib

Install dependencies with:
```bash
pip install numpy matplotlib
```

## Notes

- The simulation now uses torque as the main input, not angular acceleration.
- The controller framework is modular; see `controllers/` for examples.
- Visualization tools require Matplotlib.

*Created by Daniel Ludlow, 2025*
