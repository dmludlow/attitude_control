"""
A module for defining and handling spacecraft trajectories.
"""

import numpy as np
import src.attitude_control.utils.visualization as vis
import src.attitude_control.plant as plant

class Trajectory:
    """
    A class to represent a spacecraft trajectory.

    Attributes:
    time (np.ndarray): Array of time points.
    attitude (np.ndarray): Array of attitude quaternions at each time point.
    angular_velocity (np.ndarray): Array of angular velocities at each time point.
    """
    time: np.ndarray
    attitude: np.ndarray
    angular_velocity: np.ndarray
    
    def __init__ (self, time: np.ndarray, attitude: np.ndarray, angular_velocity: np.ndarray):
        self.time = time
        self.attitude = attitude
        self.angular_velocity = angular_velocity

    def plot_trajectory(self, dt: float):
        """
        Plots and saves the angular velocity and Euler angles over time using the visualization utilities.
        Args:
            dt (float): Time step in seconds.
        """
        import src.attitude_control.utils.visualization as vis
        # Build a list of State objects for visualization
        states = []
        for i in range(len(self.time)):
            q = self.attitude[i]
            w = self.angular_velocity[i]
            # If q is not a Quaternion object, skip or convert
            states.append(
                type('State', (), {'q': q, 'w': w})()
            )
        vis.plot_w(states, dt, filename='angular_velocity_slew.png')
        vis.plot_q(states, dt, filename='quaternion_euler_angles_slew.png')

    def get_state(self, index: int):
        """
        Returns the state at a specific index in the trajectory.

        Args:
            index (int): Index of the desired state.

        Returns:
            A simple object with attributes 'q' (quaternion) and 'w' (angular velocity).
        """
        if index < 0 or index >= len(self.time):
            raise IndexError("Index out of bounds for trajectory states.")
        q = self.attitude[index]
        w = self.angular_velocity[index]
        return plant.state.State(q,w)
    
    def merge(self, final_time: float, dt: float, other: 'Trajectory') -> 'Trajectory':
        """Merge this trajectory (A) with another (B) so that B finishes exactly at ``final_time``.

        The logic implements:
            C = A  + (optional hold segment) + shifted B

        Where:
            - A ends at tA_end = self.time[-1]
            - B is generated starting at 0 and ends at tB_end = other.time[-1]
            - We compute start_B = final_time - tB_end
            - If start_B > tA_end + dt we insert a hold segment keeping A's last state.
            - If start_B == tA_end + dt (within tolerance) we append directly.

        Args:
            final_time (float): Absolute time at which the *end* of B must align.
            dt (float): Time step.
            other (Trajectory): The new trajectory segment (B) whose local time starts at 0.

        Returns:
            Trajectory: Combined trajectory meeting the final_time constraint.

        Raises:
            ValueError: If final_time is not strictly greater than current end or there is
                        insufficient room to place B without overlap.
        """
        # Tolerances for floating point comparisons
        tol = 1e-9 * max(1.0, abs(final_time))
        tA_end = self.time[-1]

        if final_time <= tA_end + tol:
            raise ValueError("final_time must be greater than the end time of the current trajectory.")

        # Duration of 'other' (its last sample time). Assumes other.time starts at 0.
        tB_end = other.time[-1]
        if abs(other.time[0]) > tol:
            raise ValueError("Other trajectory must start at time 0.")

        # Compute when B must start so it ends exactly at final_time
        start_B = final_time - tB_end

        # Minimum feasible start time (next sample after current end)
        min_start = tA_end + dt
        if start_B < min_start - tol:
            raise ValueError("Not enough time to fit new trajectory before final_time without overlap.")

        # Build hold / filler segment if there's a gap
        if start_B > min_start + tol:
            # If possible, create a hold segment to fill the gap
            # Filler time array to gap the two segments
            filler_times = np.arange(min_start, start_B, dt)
            # Attitude array likely dtype=object, simple tile is fine
            if len(filler_times) > 0:
                # Keep attitude array 1D (len,) of Quaternion objects. np.tile would create shape (n,1)
                # leading to concatenate dimension mismatch with existing 1D arrays.
                filler_attitude = np.full(len(filler_times), self.attitude[-1], dtype=object)
                filler_w = np.tile(self.angular_velocity[-1], (len(filler_times), 1))
            else:
                filler_attitude = np.empty((0,), dtype=object)
                filler_w = np.empty((0, self.angular_velocity.shape[1]))
        else:
            filler_times = np.array([], dtype=float)
            filler_attitude = np.empty((0,), dtype=object)
            filler_w = np.empty((0, self.angular_velocity.shape[1]))

        # Shift B's time axis
        shifted_time_B = other.time + start_B

        # Concatenate
        if len(filler_times) == 0:
            # If no filler, just A + shifted B
            new_time = np.concatenate((self.time, shifted_time_B))
            new_attitude = np.concatenate((self.attitude, other.attitude))
            new_angular_velocity = np.concatenate((self.angular_velocity, other.angular_velocity))
        else:
            # With filler: A + filler + shifted B
            new_time = np.concatenate((self.time, filler_times, shifted_time_B))
            new_attitude = np.concatenate((self.attitude, filler_attitude, other.attitude))
            new_angular_velocity = np.concatenate((self.angular_velocity, filler_w, other.angular_velocity))

        # Final sanity check
        if abs(new_time[-1] - final_time) > 2*tol:
            raise RuntimeError("Merged trajectory final time mismatch.")

        return Trajectory(new_time, new_attitude, new_angular_velocity)