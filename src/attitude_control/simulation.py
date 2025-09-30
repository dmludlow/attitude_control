"""
This file contains a class to run the main simulation loop for a spacecraft attitude control system.
"""

# ISSUE WITH RANDOM FLOATING W AND A MAX VALUES

import numpy as np
import copy
from typing import Sequence
import src.attitude_control.plant.spacecraft as sc
import src.attitude_control.utils.visualization as vis
import src.attitude_control.plant.state as st
import src.attitude_control.plant.quaternion as qm
import src.attitude_control.controllers as ctrl
import src.attitude_control.command_shaping as cmd

class Simulation:

    # Time step in seconds
    dt: float
    # Time for the simulation to run in seconds.
    time: float 
    # List of spacecraft to be simulated
    crafts: list[sc.Spacecraft]

    def __init__(self, dt: float, time: float, crafts: Sequence[sc.Spacecraft]):
        self.dt = dt
        self.time = time
        # Ensure we store a mutable list of Spacecraft objects
        self.crafts = list(crafts)

    def run(self):
        """
        Runs the main simulation loop for the specified time and time step.
        """
         # Total simulation steps
        steps = int(self.time / self.dt)
        # Generate time array
        t = np.arange(0, self.time, self.dt)

        # Generate trajectories for each spacecraft if they have a state goal defined
        # This should maybe be moved to slewgenerator.py
        for craft in self.crafts:
            # Updating state used to plan trajectory without modifying the actual spacecraft state
            traj_plan_state = copy.deepcopy(craft.state)

            # Expect craft.state_goal structured as [states_array, times_array]
            if craft.state_goal is None or len(craft.state_goal) != 2:
                continue
            states_array = craft.state_goal[0]
            times_array = craft.state_goal[1]

            # Loop by index (assumes equal length arrays)
            for i in range(len(states_array)):
                goal_state = states_array[i]
                goal_time = times_array[i]
                slew_generator = cmd.slew_generator.Slew(
                    initial_state = traj_plan_state,
                    desired_state = goal_state,
                    dt = self.dt,
                    w_max = 0.1,  # Max angular velocity (rad/s)
                    a_max = 0.05  # Max angular acceleration (rad/s^2)
                )

                # Generate the new trajectory segment
                new_trajectory = slew_generator.generate_trap()

                # Merge with existing trajectory if it exists
                if craft.trajectory is None:
                    # First trajectory segment: ensure timeline starts at 0 and holds initial state until maneuver start
                    segment_duration = new_trajectory.time[-1]
                    tol = 1e-9 * max(1.0, abs(goal_time))
                    if goal_time + tol < segment_duration:
                        raise ValueError(
                            f"First segment duration ({segment_duration:.6f}) exceeds arrival time ({goal_time:.6f})."
                        )
                    # Compute when this maneuver must start so it ends at goal_time
                    start_B = goal_time - segment_duration
                    if start_B < 0:
                        start_B = 0.0  # Already validated above but safe clamp
                    if start_B > 0:
                        # Build filler hold segment from t=0 to start_B (exclusive of start_B)
                        filler_times = np.arange(0.0, start_B, self.dt)
                        # Build attitude / w arrays; attitude is object array
                        if len(filler_times) > 0:
                            # Attitude trajectory arrays are 1D object arrays of Quaternion objects.
                            # Using np.tile on a Quaternion object produces a 2D (n,1) array which then
                            # causes a dimension mismatch on concatenate with the existing 1D attitude array.
                            # Use np.full to keep a flat (n,) object array instead.
                            filler_att = np.full(len(filler_times), traj_plan_state.q, dtype=object)
                            filler_w = np.tile(traj_plan_state.w, (len(filler_times), 1))
                        else:
                            filler_att = np.empty((0,), dtype=object)
                            filler_w = np.empty((0, new_trajectory.angular_velocity.shape[1]))
                        # Shift maneuver times
                        maneuver_time = new_trajectory.time + start_B
                        # Concatenate
                        combined_time = np.concatenate((filler_times, maneuver_time))
                        combined_att = np.concatenate((filler_att, new_trajectory.attitude))
                        combined_w = np.concatenate((filler_w, new_trajectory.angular_velocity))
                        craft.trajectory = cmd.trajectory.Trajectory(combined_time, combined_att, combined_w)
                    else:
                        # Starts immediately at t=0, just shift if needed (shouldn't be for start_B=0)
                        craft.trajectory = new_trajectory
                else:
                    # Treat goal_time as ARRIVAL (end) time for this segment
                    segment_duration = new_trajectory.time[-1]
                    last_time = craft.trajectory.time[-1]
                    # Feasibility: start_B = goal_time - segment_duration must be >= last_time + dt
                    if goal_time - segment_duration < last_time + self.dt:
                        raise ValueError(
                            f"Arrival time {goal_time} too early for segment (needs at least "
                            f"{last_time + self.dt + segment_duration:.6f})."
                        )
                    craft.trajectory = craft.trajectory.merge(
                        final_time = goal_time,
                        dt = self.dt,
                        other = new_trajectory
                    )

                # Update planning state for next goal
                traj_plan_state = goal_state

        # Run the simulation for each spacecraft
        for craft in self.crafts:
            for i in range(steps):
                # Optional: Add some random disturbance torque
                random_disturbance = np.random.normal(0, 2, 3)

                # Get the desired state from the trajectory at the current time step
                if craft.trajectory is not None and i < len(craft.trajectory.time):
                    # Grabs the trajectory state
                    state_desired = craft.trajectory.get_state(i)
                elif craft.trajectory is not None:
                    # If past the end of the trajectory, hold the last state
                    state_desired = st.State(craft.trajectory.attitude[-1], craft.trajectory.angular_velocity[-1])
                else:
                    # No trajectory, hold current state
                    state_desired = craft.state 

                # Compute control torque using the PD controller
                T_cur = craft.controller.torque(
                    craft.state,
                    state_desired,
                    craft.I,
                    self.dt
                )

                # Update the spacecraft state with the computed torque
                craft.step(T_cur + random_disturbance, self.dt)

                # Store a copy of the current state in history
                craft.state_history = np.append(craft.state_history, copy.deepcopy(craft.state))

        # Announce completion
        print("Simulation complete.")

    def visualize(self):
        # Visualize the results of the simulation
        for idx, craft in enumerate(self.crafts):
            vis.plot_w(craft.state_history, self.dt, f'craft_{idx+1}_angular_velocity.png')
            vis.plot_q(craft.state_history, self.dt, f'craft_{idx+1}_quaternion_euler_angles.png')