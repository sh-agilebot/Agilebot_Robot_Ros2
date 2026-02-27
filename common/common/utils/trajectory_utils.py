"""
Copyright © 2026 Agilebot Robotics Ltd. All rights reserved.
Instruction:
This file defines the interplate function for the trajectory.
It inputs the trajectory and the time interval, and outputs the interpolated trajectory.
    - Quintic polynomial interpolation.
    - B-spline interpolation.
"""

import csv
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Optional, Union

import numpy as np
import pandas as pd
from Agilebot.IR.A.arm import Arm
from Agilebot.IR.A.file_manager import ROBOT_TMP, FileManager
from Agilebot.IR.A.sdk_types import (
    RobotStatusEnum,
    ServoStatusEnum,
)
from Agilebot.IR.A.status_code import StatusCodeEnum
from matplotlib import pyplot as plt
from common.utils.arm_utils import (
    prepare_offline_trajectory_compat,
    wait_for_transform_ready,
)
from rclpy.serialization import deserialize_message, serialize_message
from scipy.interpolate import make_interp_spline
from trajectory_msgs.msg import JointTrajectory

try:
    from scipy.interpolate import make_interp_spline
except Exception as e:
    raise ImportError("scipy is required. Install with `pip install scipy`.") from e

try:
    from dtw import dtw
except Exception as e:
    raise ImportError(
        "dtw is required. Install with `pip install pandas dtw-python`."
    ) from e


def save_joint_trajectory_to_file(trajectory: JointTrajectory, filename: str):
    """
    Save the serialized JointTrajectory message to a file directly
    Args:
        trajectory (JointTrajectory): The trajectory to save
        filename (str): The name of the file to save to
    """
    serialized_data = serialize_message(trajectory)
    with open(filename, "wb") as f:
        f.write(serialized_data)


def load_joint_trajectory_from_file(filename: str) -> JointTrajectory:
    """
    Load a serialized JointTrajectory message from a file directly

    Args:
        filename (str): The name of the file to load from

    Returns:
        JointTrajectory: The loaded trajectory
    """
    with open(filename, "rb") as f:
        data = f.read()
    return deserialize_message(data, JointTrajectory)


def read_and_coarse_sample(
    csv_path: str,
    sample_dt: float = 0.1,
    ts_col: str = "ts",
    csv_coarse_path: str = "coarse.csv",
) -> pd.DataFrame:
    """
    read 1ms sampling rate CSV and coarse sample by sample_dt (s).
    This function down‑samples a tested‑trajectory CSV into a coarse‑sampling‑rate CSV for debugging.
    Args:
        csv_path (str):   CSV file path
        sample_dt (float): Coarse sampling interval, in seconds (default 0.1s)
        ts_col (str):     Timestamp column name (default "ts")
        csv_coarse_path (str): Coarse sampling CSV file path

    Returns:
        pd.DataFrame: Coarse sampling DataFrame
    """
    # 1. read CSV
    df = pd.read_csv(csv_path)

    # 2. compute step: sample_dt seconds / 0.001 seconds = how many rows
    step = int(round(sample_dt / 0.001))
    if step < 1:
        raise ValueError(
            f"sample_dt={sample_dt} too small, step must be >= 1 original sampling point."
        )

    # 3. Slice the DataFrame by position to take every step row
    df_coarse = df.iloc[::step].reset_index(drop=True)

    # Optional: If you want to make sure the last moment is also taken, you can do this:
    if df_coarse[ts_col].iloc[-1] != df[ts_col].iloc[-1]:
        df_coarse = pd.concat([df_coarse, df.tail(1)], ignore_index=True)

    df_coarse.to_csv(csv_coarse_path, index=False)
    return df_coarse


def run_trajectory(arm: Arm, local_file_path: str) -> bool:
    """
    Uploads a CSV trajectory file to the robot controller, converts it into the
    robot-specific trajectory format, and executes it in offline mode.

    Parameters:
        arm (Arm):                Instance of the robot arm interface.
        local_file_path (str):    Path to the local CSV file containing trajectory waypoints.

    Returns:
        bool: True if the trajectory was executed successfully, False otherwise.
    """

    file_name = os.path.basename(local_file_path)
    file_manager = FileManager(arm.controller_ip)

    # 1. Upload the CSV file to the robot controller
    if (
        file_manager.upload(local_file_path, ROBOT_TMP, overwriting=True)
        != StatusCodeEnum.OK
    ):
        return False
    print("File uploaded successfully")

    # 2. Convert CSV to robot trajectory format
    file_dir, ret = arm.trajectory.transform_csv_to_trajectory(file_name, io_flag="2")
    if ret != StatusCodeEnum.OK:
        print(f"Transformation error: {ret}")
        return False
    print(f"Converted trajectory file saved at: {file_dir}")

    transformed_file_name = os.path.basename(file_dir)

    # 3. Poll until conversion completes or fails
    if not wait_for_transform_ready(arm, transformed_file_name):
        print("Final transform status: failed")
        return False
    print("CSV-to-trajectory conversion succeeded")

    # 5. Set the offline trajectory file on the controller
    if (
        arm.trajectory.set_offline_trajectory_file(transformed_file_name)
        != StatusCodeEnum.OK
    ):
        print("Failed to set trajectory file")
        return False
    print("Trajectory file set successfully")

    # 6. Prepare the controller for offline execution
    if prepare_offline_trajectory_compat(arm) != StatusCodeEnum.OK:
        print("Failed to prepare offline trajectory")
        return False
    print("Offline trajectory prepared successfully")

    # 7. Wait until the robot and servos are idle (ready to run)
    while True:
        robot_status, ret_robot = arm.get_robot_status()
        servo_status, ret_servo = arm.get_servo_status()

        if (
            ret_robot != StatusCodeEnum.OK
            or ret_servo != StatusCodeEnum.OK
            or (
                robot_status == RobotStatusEnum.ROBOT_IDLE
                and servo_status == ServoStatusEnum.SERVO_IDLE
            )
        ):
            break
        time.sleep(0.5)
    print("Robot is ready for execution")

    # 8. Execute the offline trajectory
    success = arm.trajectory.execute_offline_trajectory() == StatusCodeEnum.OK
    if success:
        print("Trajectory executed successfully")
    else:
        print("Trajectory execution failed")
    return success


def plot_joint_metrics(csv_path):
    """
    read and plot joint metrics from CSV file. Include position, velocity, acceleration, and jerk.
    Args:
            csv_path (str): CSV file path, should include 'ts',
                            'pts_J1'…'pts_J6',
                            'vel_J1'…'vel_J6',
                            'acc_J1'…'acc_J6',
    """
    # read csv
    df = pd.read_csv(csv_path)
    # time series
    t = df["ts"].values

    metrics = {
        "pts": "Position (rad)",
        "vel": "Velocity (rad/s)",
        "acc": "Acceleration (rad/s²)",
        "jerk": "Jerk (rad/s³)",
    }

    for metric, ylabel in metrics.items():
        plt.figure()
        for j in range(1, 7):  # six joints,from 1 to 6
            y = df[f"{metric}_J{j}"].values
            plt.scatter(t, y, s=0.2, label=f"Joint {j}")
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.title(f"{ylabel} vs Time")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

    plt.show()


def quincic_polynomial_interpolation(
    trajectory: Union[JointTrajectory, pd.DataFrame],
    output_csv_path: str = "trajectory.csv",
) -> bool:
    """
    Use quintic polynomial interpolation to generate a trajectory.
    Notes: If the input trajectory is long, the interpolation may take a long time and consume a lot of memory.
    Args:
         trajectory (Union[JointTrajectory, pd.DataFrame]): trajectory data, can be a JointTrajectory or a pandas DataFrame.
         output_csv_path (str): output trajectory CSV file path.
    Returns:
         bool: True if success, False if failed.
    """
    # 1.extract data from trajectory or DataFrame
    if isinstance(trajectory, JointTrajectory):  # ROS2 JointTrajectory
        pts = trajectory.points
        ts_via = np.array(
            [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]
        )
        positions = np.array([p.positions for p in pts])
        velocities = np.array([p.velocities for p in pts])
        accelerations = np.array([p.accelerations for p in pts])
    elif isinstance(trajectory, pd.DataFrame):  # pandas.DataFrame
        df = trajectory
        ts_via = df["ts"].to_numpy()
        positions = df[[f"pts_J{i+1}" for i in range(6)]].to_numpy()
        velocities = df[[f"vel_J{i+1}" for i in range(6)]].to_numpy()
        accelerations = df[[f"acc_J{i+1}" for i in range(6)]].to_numpy()
    else:
        raise TypeError("trajectory must be a JointTrajectory or a pandas DataFrame")

    J = positions.shape[1]  # degrees of freedom, i.e. number of joints, here 6
    # Total number of sample points = total time (i.e. the final timestamp) * sampling rate + 1
    total_points = int(np.round(ts_via[-1] * 1000)) + 1
    # time series
    ts = (
        np.arange(total_points) / 1000.0
    )  # shape (T,).T is the total number of sample points

    # 2.compute the coefficients of each segment polynomial
    # M= T-1, number of segments, i.e. number of key points minus one
    t0 = ts_via[:-1]  # shape (M,). t0 is the start time of each segment
    t1 = ts_via[1:]  # shape (M,). t1 is the end time of each segment
    delta_t = t1 - t0  # shape (M,). delta_T is the duration of each segment

    delta_p = (
        positions[1:] - positions[:-1]
    )  # (M, J). delta_p is the position change of each joint in each segment
    v0 = velocities[
        :-1
    ]  # (M, J). v0 is the start velocity of each joint in each segment
    v1 = velocities[1:]  # (M, J). v1 is the end velocity of each joint in each segment
    a0 = accelerations[
        :-1
    ]  # (M, J). a0 is the start acceleration of each joint in each segment
    a1 = accelerations[
        1:
    ]  # (M, J). a1 is the end acceleration of each joint in each segment

    # compute coefficients A0..A5 (broadcast to shape (M,J))
    A0 = positions[:-1]
    A1 = v0
    A2 = a0 / 2
    A3 = (
        20 * delta_p
        - (8 * v1 + 12 * v0) * delta_t[:, None]
        - (3 * a0 - a1) * (delta_t[:, None] ** 2)
    ) / (2 * (delta_t[:, None] ** 3))
    A4 = (
        -30 * delta_p
        + (14 * v1 + 16 * v0) * delta_t[:, None]
        + (3 * a0 - 2 * a1) * (delta_t[:, None] ** 2)
    ) / (2 * (delta_t[:, None] ** 4))
    A5 = (
        12 * delta_p
        - 6 * (v1 + v0) * delta_t[:, None]
        + (a1 - a0) * (delta_t[:, None] ** 2)
    ) / (2 * (delta_t[:, None] ** 5))

    # 3.Segment index
    # Assume the data to be interpolated contains data for 11 time points, which means there are 11-1=10 segments. Therefore, the interpolation result for each time point needs to be mapped to the corresponding segment.
    # For each ts, find the segment it belongs to, i.e., ts ∈ [t0[i], t1[i]), where seg_idx represents the index of the segment.
    seg_idx = np.searchsorted(ts_via, ts, side="right") - 1
    seg_idx = np.clip(
        seg_idx, 0, delta_t.shape[0] - 1
    )  #  # Prevent out-of-bounds by mapping points outside the start and end to valid segments (segment index 0 or M-1)
    dt = (ts - t0[seg_idx])[
        ..., None
    ]  # shape (T,1),# Used for broadcasting to J dimensions. Represents the time change Δt for each segment, Δt = sample point time - segment start time.

    # 4.Vectorized Calculation
    # position: a0 + a1*dt + a2*dt² + … + a5*dt⁵
    pos = (
        A0[seg_idx]
        + A1[seg_idx] * dt
        + A2[seg_idx] * dt**2
        + A3[seg_idx] * dt**3
        + A4[seg_idx] * dt**4
        + A5[seg_idx] * dt**5
    )  # shape (T, J)

    vel = (
        A1[seg_idx]
        + 2 * A2[seg_idx] * dt
        + 3 * A3[seg_idx] * dt**2
        + 4 * A4[seg_idx] * dt**3
        + 5 * A5[seg_idx] * dt**4
    )

    acc = (
        2 * A2[seg_idx]
        + 6 * A3[seg_idx] * dt
        + 12 * A4[seg_idx] * dt**2
        + 20 * A5[seg_idx] * dt**3
    )

    jerk = 6 * A3[seg_idx] + 24 * A4[seg_idx] * dt + 60 * A5[seg_idx] * dt**2

    # 5. Write to CSV
    header = (
        ["ts"]
        + [f"pts_J{i+1}" for i in range(J)]
        + [f"vel_J{i+1}" for i in range(J)]
        + [f"acc_J{i+1}" for i in range(J)]
        + [f"jerk_J{i+1}" for i in range(J)]
        + ["do_port", "do_state"]
    )

    do_port = -np.ones(total_points, dtype=int)
    do_state = np.zeros(total_points, dtype=int)

    # Prepare column data as a dict for DataFrame (keeps column order defined by `header`)
    data_dict = {"ts": ts}
    for j in range(J):
        data_dict[f"pts_J{j+1}"] = pos[:, j]
    for j in range(J):
        data_dict[f"vel_J{j+1}"] = vel[:, j]
    for j in range(J):
        data_dict[f"acc_J{j+1}"] = acc[:, j]
    for j in range(J):
        data_dict[f"jerk_J{j+1}"] = jerk[:, j]

    data_dict["do_port"] = do_port
    data_dict["do_state"] = do_state

    # Create DataFrame and write out (float_format )
    df_out = pd.DataFrame(data_dict, columns=header)
    df_out.to_csv(output_csv_path, index=False, float_format="%.6f")

    # print(f"finished writing to {output_csv_path}")
    return True


def trajectory_interpolate(
    trajectory: Union[JointTrajectory, pd.DataFrame],
    output_csv_path: str = "trajectory.csv",
    mode="quintic",
) -> bool:
    """
    Interpolate the trajectory and generate a trajactory CSV file.
    Notes: If the input trajectory is long, the interpolation may take a long time and consume a lot of memory.
    Args:
         trajectory (Union[JointTrajectory, pd.DataFrame]): trajectory data, can be a JointTrajectory or a pandas DataFrame.
         output_csv_path (str): output trajectory CSV file path.
         mode (str): interpolation mode, can be "quintic" or "spline".Default is "quintic".
    """
    if mode == "quintic":
        return quincic_polynomial_interpolation(trajectory, output_csv_path)
    elif mode == "spline":
        return bspline_resample(trajectory, output_csv_path)
    else:
        raise ValueError(f"Unsupported interpolation mode: {mode}")


def _fit_spline_for_joint(args):
    """
    Helper for ProcessPoolExecutor: fits a spline for a single joint and returns
    (j_index, pos, vel, acc, jerk, k_used).
    args: tuple (j, ts, y, v0, vN, a0, aN, clamp_vel, clamp_acc, sample_ts_new, ensure_endpoint_included)
    """
    (
        j,
        ts,
        y,
        v0,
        vN,
        a0,
        aN,
        clamp_vel,
        clamp_acc,
        ts_new,
    ) = args

    left_bc = []
    right_bc = []

    if clamp_vel and (v0 is not None) and (vN is not None):
        if np.isfinite(v0) and np.isfinite(vN):
            left_bc.append((1, float(v0)))
            right_bc.append((1, float(vN)))

    if clamp_acc and (a0 is not None) and (aN is not None):
        if np.isfinite(a0) and np.isfinite(aN):
            left_bc.append((2, float(a0)))
            right_bc.append((2, float(aN)))

    use_vel = any(o == 1 for o, _ in (left_bc + right_bc))
    use_acc = any(o == 2 for o, _ in (left_bc + right_bc))
    k = 5 if (use_vel and use_acc) else 3

    N = len(ts)
    if N < (k + 1):
        raise ValueError(
            f"[joint {j}] Not enough points N={N} for spline degree k={k}. Need >= {k+1}."
        )

    bc_type = (
        (left_bc if left_bc else [], right_bc if right_bc else [])
        if (left_bc or right_bc)
        else "natural"
    )
    spline = make_interp_spline(ts, y, k=k, bc_type=bc_type)

    pos = spline(ts_new)
    vel = spline(ts_new, 1)
    acc = spline(ts_new, 2)
    jerk = spline(ts_new, 3)

    return (j, pos, vel, acc, jerk, k)


def bspline_resample_from_dataframe(
    df: pd.DataFrame,
    output_csv_path: str,
    sample_hz: float = 1000.0,
    clamp_endpoint_velocities: bool = True,
    clamp_endpoint_accelerations: bool = True,
    ensure_endpoint_included: bool = True,
    mode: str = "auto",  # "auto" | "vectorized" | "parallel"
    max_workers: Optional[int] = None,
) -> bool:
    """
    Resamples a trajectory from a pandas DataFrame using B-spline interpolation.

    This function takes a DataFrame containing time-stamped joint positions and optionally
    velocities and accelerations, then generates a new, denser trajectory by fitting
    B-splines to the data. It supports different parallelization modes to optimize performance.

    Args:
        df (pd.DataFrame): The input DataFrame containing trajectory data. It must include
            a 'ts' column for timestamps and 'pts_J#' columns for joint positions.
            'vel_J#' and 'acc_J#' columns are optional but required for clamping.
        output_csv_path (str): The file path where the resampled trajectory
            will be saved as a CSV file.
        sample_hz (float, optional): The desired sampling frequency in Hertz for the
            output trajectory. Defaults to 1000.0.
        clamp_endpoint_velocities (bool, optional): If True, the spline's endpoint
            velocities (first derivative) will be clamped to the values provided in
            the DataFrame's 'vel_J#' columns. This requires 'vel_J#' columns to be present.
            Defaults to True.
        clamp_endpoint_accelerations (bool, optional): If True, the spline's endpoint
            accelerations (second derivative) will be clamped to the values provided
            in the DataFrame's 'acc_J#' columns. This requires 'acc_J#' columns to be present.
            Defaults to True.
        ensure_endpoint_included (bool, optional): If True, ensures the last timestamp
            of the original trajectory is included in the resampled output, even if
            it doesn't perfectly align with the `sample_hz`. Defaults to True.
        mode (str, optional): The execution strategy for resampling.
            - "auto": Tries "vectorized" mode first. If endpoint clamping conditions
              are not met (e.g., missing velocity/acceleration data), it falls back to "parallel" mode.
            - "vectorized": Fits a single vector-valued spline for all joints simultaneously.
              This is the fastest method but requires consistent boundary conditions across all joints.
            - "parallel": Fits a separate spline for each joint and processes them in parallel
              using a process pool.
            Defaults to "auto".
        max_workers (Optional[int], optional): The maximum number of worker processes
            to use in "parallel" mode. If None, it will use the number of CPU cores.
            Defaults to None.


    Returns:
        bool: True if the resampling was successful, False otherwise.
    """
    if "ts" not in df.columns:
        raise ValueError("DataFrame must contain 'ts' column (seconds).")

    ts = df["ts"].to_numpy(dtype=float)
    if ts.ndim != 1 or len(ts) < 2:
        raise ValueError("ts must be a 1-D array with at least two entries.")
    if not np.all(np.diff(ts) > 0):
        raise ValueError("Times in 'ts' must be strictly increasing and unique.")

    pts_cols = [c for c in df.columns if c.startswith("pts_J")]
    if len(pts_cols) == 0:
        raise ValueError("No joint position columns 'pts_J#' found in DataFrame.")
    pts_cols = sorted(
        pts_cols, key=lambda s: int("".join(filter(str.isdigit, s)) or -1)
    )
    J = len(pts_cols)

    positions = df[pts_cols].to_numpy(dtype=float)  # (N, J)

    vel_cols = [f"vel_J{i+1}" for i in range(J)]
    acc_cols = [f"acc_J{i+1}" for i in range(J)]
    have_vel_cols = all(c in df.columns for c in vel_cols)
    have_acc_cols = all(c in df.columns for c in acc_cols)
    velocities = df[vel_cols].to_numpy(dtype=float) if have_vel_cols else None
    accelerations = df[acc_cols].to_numpy(dtype=float) if have_acc_cols else None

    t_start = float(ts[0])
    t_end = float(ts[-1])
    dt = 1.0 / float(sample_hz)
    ts_new = np.arange(t_start, t_end, dt)
    if ensure_endpoint_included:
        if ts_new.size == 0 or ts_new[-1] < t_end - 1e-12:
            ts_new = np.append(ts_new, t_end)
    ts_new = ts_new.astype(float)

    N = len(ts)
    pos_new = np.zeros((len(ts_new), J), dtype=float)
    vel_new = np.zeros((len(ts_new), J), dtype=float)
    acc_new = np.zeros((len(ts_new), J), dtype=float)
    jerk_new = np.zeros((len(ts_new), J), dtype=float)

    # Helper: check if vectorized path is possible
    def _vectorizable():
        # need velocity & acceleration columns present or both unused,
        # and endpoint values must be finite for all joints when used.
        use_vel = clamp_endpoint_velocities and have_vel_cols
        use_acc = clamp_endpoint_accelerations and have_acc_cols
        if not (use_vel or use_acc):
            return True  # no endpoint derivatives -> vectorizable
        # if using vel/acc, require columns exist for all joints
        if use_vel and velocities is None:
            return False
        if use_acc and accelerations is None:
            return False
        # ensure endpoint values are finite arrays for all joints
        if use_vel:
            if not np.all(np.isfinite(velocities[0, :])) or not np.all(
                np.isfinite(velocities[-1, :])
            ):
                return False
        if use_acc:
            if not np.all(np.isfinite(accelerations[0, :])) or not np.all(
                np.isfinite(accelerations[-1, :])
            ):
                return False
        return True

    chosen_mode = mode
    if mode == "auto":
        chosen_mode = "vectorized" if _vectorizable() else "parallel"

    # VECTORIZE: single vector-valued spline y shape (N, J)
    if chosen_mode == "vectorized":
        left_bc = []
        right_bc = []
        use_vel = clamp_endpoint_velocities and have_vel_cols
        use_acc = clamp_endpoint_accelerations and have_acc_cols

        if use_vel:
            v0 = velocities[0, :].astype(float)
            vN = velocities[-1, :].astype(float)
            left_bc.append((1, v0))
            right_bc.append((1, vN))
        if use_acc:
            a0 = accelerations[0, :].astype(float)
            aN = accelerations[-1, :].astype(float)
            left_bc.append((2, a0))
            right_bc.append((2, aN))

        use_both = use_vel and use_acc
        k = 5 if use_both else 3
        if N < (k + 1):
            raise ValueError(
                f"Not enough input points N={N} for vectorized k={k} (need >= {k+1})."
            )

        bc_type = (
            (left_bc if left_bc else [], right_bc if right_bc else [])
            if (left_bc or right_bc)
            else "natural"
        )
        # positions shape (N, J) -> vector-valued spline
        spline = make_interp_spline(ts, positions, k=k, bc_type=bc_type)

        pos_all = spline(ts_new)  # shape (len(ts_new), J)
        vel_all = spline(ts_new, 1)
        acc_all = spline(ts_new, 2)
        jerk_all = spline(ts_new, 3)

        pos_new[:, :] = pos_all
        vel_new[:, :] = vel_all
        acc_new[:, :] = acc_all
        jerk_new[:, :] = jerk_all

        spline_k_used = k

    elif chosen_mode == "parallel":
        # prepare args per joint
        args_list = []
        for j in range(J):
            v0 = velocities[0, j] if (velocities is not None) else None
            vN = velocities[-1, j] if (velocities is not None) else None
            a0 = accelerations[0, j] if (accelerations is not None) else None
            aN = accelerations[-1, j] if (accelerations is not None) else None
            args_list.append(
                (
                    j,
                    ts,
                    positions[:, j],
                    v0,
                    vN,
                    a0,
                    aN,
                    clamp_endpoint_velocities,
                    clamp_endpoint_accelerations,
                    ts_new,
                )
            )

        n_workers = max_workers or min(J, max(1, (os.cpu_count() or 1)))
        # use process pool for CPU bound tasks
        results = []
        with ProcessPoolExecutor(max_workers=n_workers) as exe:
            futures = {exe.submit(_fit_spline_for_joint, a): a[0] for a in args_list}
            for fut in as_completed(futures):
                res = fut.result()
                results.append(res)

        # collect results (unordered), place into arrays
        spline_k_used = None
        for j, pos, vel, acc, jerk, k in results:
            pos_new[:, j] = pos
            vel_new[:, j] = vel
            acc_new[:, j] = acc
            jerk_new[:, j] = jerk
            spline_k_used = (
                k if spline_k_used is None else spline_k_used
            )  # k should be same or we can ignore

    else:
        raise ValueError("mode must be one of 'auto','vectorized','parallel'")

    # prepare DataFrame to save
    header = (
        ["ts"]
        + [f"pts_J{i+1}" for i in range(J)]
        + [f"vel_J{i+1}" for i in range(J)]
        + [f"acc_J{i+1}" for i in range(J)]
        + [f"jerk_J{i+1}" for i in range(J)]
        + ["do_port", "do_state"]
    )
    data = {"ts": ts_new}
    metrics = [("pts", pos_new), ("vel", vel_new), ("acc", acc_new), ("jerk", jerk_new)]
    data.update(
        {
            f"{metric}_J{j+1}": values[:, j]
            for metric, values in metrics
            for j in range(J)
        }
    )

    do_port = -np.ones(len(ts_new), dtype=int)
    do_state = np.zeros(len(ts_new), dtype=int)
    data["do_port"] = do_port
    data["do_state"] = do_state

    df_out = pd.DataFrame(data, columns=header)
    df_out.to_csv(output_csv_path, index=False, float_format="%.6f")
    print(
        f"Wrote B-spline resampled trajectory to {output_csv_path} (samples: {len(ts_new)}, joints: {J}, mode={chosen_mode}, spline_k={spline_k_used})"
    )
    return True


def bspline_resample(
    trajectory: Union["JointTrajectory", pd.DataFrame, str],
    output_csv_path: str = "trajectory_bspline.csv",
    sample_hz: float = 1000.0,
    clamp_endpoint_velocities: bool = True,
    clamp_endpoint_accelerations: bool = True,
    mode: str = "auto",
    max_workers: Optional[int] = None,
) -> bool:
    """
    Resamples a given trajectory using B-splines to create a smoother, more dense trajectory.

    This function accepts trajectory data in several formats: a path to a CSV file, a pandas
    DataFrame, or a `JointTrajectory` object. It then uses B-spline interpolation to
    resample the data at a specified frequency, ensuring the output trajectory is
    smooth and adheres to a new sampling rate.

    Args:
        trajectory (Union["JointTrajectory", pd.DataFrame, str]): The input trajectory.
            It can be a file path to a CSV, a pandas DataFrame, or a `JointTrajectory` object.
        output_csv_path (str, optional): The file path where the resampled trajectory
            will be saved as a CSV. Defaults to "trajectory_bspline.csv".
        sample_hz (float, optional): The desired sampling frequency in Hertz for the
            output trajectory. Defaults to 1000.0.
        clamp_endpoint_velocities (bool, optional): If True, the spline's endpoint
            velocities will be clamped to the values provided in the input data.
            Defaults to True.
        clamp_endpoint_accelerations (bool, optional): If True, the spline's endpoint
            accelerations will be clamped to the values provided in the input data.
            Defaults to True.
        mode (str, optional): The execution strategy for resampling. Options are:
            - "auto": Automatically selects the best strategy.
            - "vectorized": Uses a single, vector-valued spline for all joints (fastest).
            - "parallel": Resamples each joint in a separate process.
            - "serial": Resamples each joint sequentially.
            Defaults to "auto".
        max_workers (Optional[int], optional): The maximum number of worker processes
            to use in "parallel" mode. If None, it will use the number of CPU cores.
            Defaults to None.

    Returns:
        None: The function saves the result to a CSV file and returns a pandas DataFrame which contains the resampled trajectory.
    """
    if isinstance(trajectory, str):
        df = pd.read_csv(trajectory)
        return bspline_resample_from_dataframe(
            df,
            output_csv_path,
            sample_hz,
            clamp_endpoint_velocities,
            clamp_endpoint_accelerations,
            True,
            mode,
            max_workers,
        )
    elif JointTrajectory is not None and isinstance(trajectory, JointTrajectory):
        pts = trajectory.points
        ts = np.array(
            [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts],
            dtype=float,
        )
        positions = np.array([p.positions for p in pts], dtype=float)
        velocities = np.array(
            [
                p.velocities if len(p.velocities) > 0 else [0.0] * positions.shape[1]
                for p in pts
            ],
            dtype=float,
        )
        accelerations = np.array(
            [
                (
                    p.accelerations
                    if hasattr(p, "accelerations") and len(p.accelerations) > 0
                    else [0.0] * positions.shape[1]
                )
                for p in pts
            ],
            dtype=float,
        )
        data = {"ts": ts}
        J = positions.shape[1]
        for j in range(J):
            data[f"pts_J{j+1}"] = positions[:, j]
        for j in range(J):
            data[f"vel_J{j+1}"] = velocities[:, j]
        for j in range(J):
            data[f"acc_J{j+1}"] = accelerations[:, j]
        df = pd.DataFrame(data)
        return bspline_resample_from_dataframe(
            df,
            output_csv_path,
            sample_hz,
            clamp_endpoint_velocities,
            clamp_endpoint_accelerations,
            True,
            mode,
            max_workers,
        )
    elif isinstance(trajectory, pd.DataFrame):
        return bspline_resample_from_dataframe(
            trajectory,
            output_csv_path,
            sample_hz,
            clamp_endpoint_velocities,
            clamp_endpoint_accelerations,
            True,
            mode,
            max_workers,
        )
    else:
        raise TypeError(
            "trajectory must be a path to CSV, pandas.DataFrame, or JointTrajectory (if available)."
        )


def calculate_trajectory_similarity(
    target_csv_path: str, interpolated_csv_path: str, algorithm_name: str
):
    """
    Calculates and compares the similarity of two robot trajectory CSV files (aligned by millisecond timestamps).

    This function loads the target and interpolated trajectories. It rounds the timestamps
    to the nearest millisecond for data alignment. Then, it calculates the Root Mean Squared Error (RMSE),
    Mean Absolute Error (MAE), and Dynamic Time Warping (DTW) distance for each joint (J1-J6)
    and kinematic parameter (position, velocity, acceleration, jerk).

    Parameters:
        target_csv_path (str): The file path to the target trajectory CSV.
        interpolated_csv_path (str): The file path to the interpolated/actual trajectory CSV.
        algorithm_name (str): The name of the algorithm used for interpolation.

    Returns:
    pandas.DataFrame: A summary table containing the similarity calculation results for all metrics and joints.
    """
    try:
        # 1. Load data
        df_target = pd.read_csv(target_csv_path)
        df_interp = pd.read_csv(interpolated_csv_path)
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}")
        return None

    # 2. Data Alignment
    print("Aligning data based on millisecond timestamps...")
    df_target["ts_ms_rounded"] = (df_target["ts"] * 1000).round().astype(int)
    df_interp["ts_ms_rounded"] = (df_interp["ts"] * 1000).round().astype(int)

    # To prevent multiple points within the same millisecond, we keep only the first data point for each millisecond.
    df_target.drop_duplicates(subset="ts_ms_rounded", keep="first", inplace=True)
    df_interp.drop_duplicates(subset="ts_ms_rounded", keep="first", inplace=True)

    # Perform an inner merge using the new millisecond timestamp column.
    df_merged = pd.merge(
        df_target, df_interp, on="ts_ms_rounded", suffixes=("_target", "_interp")
    )

    if df_merged.empty:
        print(
            "Error: The two CSV files have no common timestamps even at the millisecond level. Comparison cannot be performed."
        )
        return None

    # 3. Prepare for calculations
    results_list = []
    average_results_list = []  # New: list to store average results

    metric_map = {
        "Position": "pts",
        "Velocity": "vel",
        "Acceleration": "acc",
        "Jerk": "jerk",
    }

    print("Calculating similarity, please wait...")

    # 4. Iterate through each metric and each joint for calculation
    for metric_name, prefix in metric_map.items():
        avg_metrics = {"RMSE": [], "MAE": [], "DTW_Distance": []}
        for i in range(1, 7):
            col_target = f"{prefix}_J{i}_target"
            col_interp = f"{prefix}_J{i}_interp"

            if (
                col_target not in df_merged.columns
                or col_interp not in df_merged.columns
            ):
                print(
                    f"Warning: Column {col_target} or {col_interp} not found in the file, skipping calculation."
                )
                continue

            s_target = df_merged[col_target]
            s_interp = df_merged[col_interp]

            # Calculate metrics
            rmse = np.sqrt(np.mean((s_target - s_interp) ** 2))
            mae = np.mean(np.abs(s_target - s_interp))

            # DTW (calculated after normalization)
            s_target_norm = (s_target - s_target.mean()) / s_target.std()
            s_interp_norm = (s_interp - s_interp.mean()) / s_interp.std()
            s_target_norm = s_target_norm.fillna(0)
            s_interp_norm = s_interp_norm.fillna(0)

            try:
                alignment = dtw(s_target_norm, s_interp_norm, keep_internals=False)
                dtw_distance = alignment.distance
            except Exception as e:
                dtw_distance = np.nan
                print(f"Warning: DTW calculation failed for {metric_name} J{i}: {e}")

            # Record results for a single joint
            results_list.append(
                {
                    "Metric": metric_name,
                    "Joint": f"J{i}",
                    "RMSE": rmse,
                    "MAE": mae,
                    "DTW Distance": dtw_distance,
                }
            )

            # Record data needed for average calculation
            avg_metrics["RMSE"].append(rmse)
            avg_metrics["MAE"].append(mae)
            if not np.isnan(dtw_distance):
                avg_metrics["DTW_Distance"].append(dtw_distance)

        # After the loop, calculate and store the average for the current metric
        if avg_metrics["RMSE"]:
            average_results_list.append(
                {
                    "Metric": metric_name,
                    "Joint": "Average",
                    "RMSE": np.mean(avg_metrics["RMSE"]),
                    "MAE": np.mean(avg_metrics["MAE"]),
                    "DTW Distance": (
                        np.mean(avg_metrics["DTW_Distance"])
                        if avg_metrics["DTW_Distance"]
                        else np.nan
                    ),
                }
            )

    # Append all average results to the end of the main results list
    results_list.extend(average_results_list)

    # 5. Convert results to DataFrame and print
    results_df = pd.DataFrame(results_list)
    results_df = results_df[["Metric", "Joint", "RMSE", "MAE", "DTW Distance"]]

    pd.set_option("display.max_rows", None)
    pd.set_option("display.width", 120)

    print("\n" + "=" * 60)
    print(f"Trajectory Similarity Analysis Results: { algorithm_name} vs Baseline")
    print("=" * 60)
    print("Explanation:")
    print(" - Data has been aligned at the millisecond level.")
    print(
        " - RMSE: Root Mean Squared Error; smaller values indicate higher similarity."
    )
    print(" - MAE: Mean Absolute Error; smaller values indicate higher similarity.")
    print(
        " - DTW Distance: Dynamic Time Warping distance (based on normalized data); smaller values indicate more similar shapes."
    )
    print("-" * 60)

    print(results_df.to_string(index=False))

    print("=" * 60)

    return results_df


if __name__ == "__main__":

    # Notes: if you don't have the sample.csv file, you can find it in the "assets" folder.

    # 1. (Optional) Generate a coarse-sampled test file from the original CSV
    # read_and_coarse_sample(
    #     csv_path="sample.csv",
    #     csv_coarse_path="traj.csv",
    #     sample_dt=0.1
    # )

    # 2. (Optional) Load a binary trajectory file into memory
    # traj = load_joint_trajectory_from_file("trajectory.bin")
    # print("Original trajectory message:", traj)

    # 3. Load the coarse-sampled trajectory from CSV into a DataFrame
    traj = pd.read_csv("traj.csv")

    # 4. Interpolate the trajectory data to 1 ms resolution
    print("Starting interpolation to 1 ms resolution...")
    start_time = time.time()
    mode = "spline"  # "quintic" or "spline"
    # You can switch between different interpolation implementations:
    trajectory_interpolate(traj, "result.csv", mode=mode)
    elapsed = time.time() - start_time
    print(f"Interpolation completed in {elapsed:.2f} seconds")

    # 5. Plot joint position, velocity, acceleration, and jerk over time
    plot_joint_metrics("result.csv")

    # 6. (Optional) Connect to the robot and execute the generated trajectory
    # arm = Arm()
    # arm.connect()
    # run_trajectory(arm, "trajectory.csv")

    # 7. Calculate and print the similarity metrics
    calculate_trajectory_similarity(
        target_csv_path="sample.csv",
        interpolated_csv_path="result.csv",
        algorithm_name=mode,
    )
