# Attitude Determination and Control System (ADCS) Simulator

This project simulates the attitude dynamics and control of spacecraft using Python. It is modular, extensible, and designed for both quick experimentation and future scalability.

## Features

- **State:** Propagation:** Simulates spacecraft attitude and angular velocity using quaternion math.
- **Dynamics:** Models rotational motion based on applied torques and spacecraft inertia.
- **Control:** (Planned/Partial) Framework for implementing attitude control laws.
- **Visualization:** Tools for plotting and visualizing attitude over time.
- **Testing:** Basic unit tests for core modules.

## Folder Structure

```
ACS_sim/
├── control.py           # Attitude control law implementations (stub/partial)
├── dynamics.py          # Spacecraft rotational dynamics
├── quaternion_math.py   # Quaternion operations and math utilities
├── sim.py               # Main simulation loop and setup
├── spacecraft.py        # Spacecraft class (inertia, state, torque application)
├── state.py             # State representation (position, velocity, attitude, etc.)
├── testing.py           # Unit tests for modules
├── visualization.py     # Visualization and plotting tools
└── __pycache__/         # Python cache files (ignore)
```

*Created by Daniel Ludlow, 2025*
