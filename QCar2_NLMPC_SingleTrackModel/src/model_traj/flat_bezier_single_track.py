#!/usr/bin/env python3
"""Generate a QCar2 single-track piecewise degree-5 Bezier trajectory from [X, Y] waypoints.

Single-track model variables:
    state x = [X, Y, psi, vx]
    input u = [delta, ax]

User-defined data follow the waypoint-geometry formulation:
    WAYPOINTS      = [X, Y] positions only
    THETA_START    = imposed heading at the first waypoint
    THETA_END      = imposed heading at the last waypoint
    SEGMENT_TIMES  = travelling time of each segment

The remaining boundary quantities psi, vx, delta, and ax are computed from
waypoint geometry and segment times before constructing degree-5 Bezier curves.
"""

from __future__ import annotations

from pathlib import Path
from math import comb

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------- User parameters -----------------------------
LWB = 0.25725            # wheelbase [m]
DT = 0.02                # sample time [s]
DELTA_MAX = 0.5236       # steering angle limit [rad]
AX_MAX = 1.0             # longitudinal acceleration limit [m/s^2]

DEFAULT_SPEED = 0.12
MIN_SEGMENT_TIME = 1.0
TIME_SAFETY_FACTOR = 1.25
ZERO_ENDPOINT_STEERING = True

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[2] / "data" / "reference"

# Edit this list in normal use: [X, Y] waypoint positions only.
WAYPOINTS = np.array(
    [
        [0.0, 0.0],
        [-1.0, 0.4],
        [-1.5, 1.5],
        [-1.0, 2.6],
        [0.0, 3.0],
        [1.0, 2.6],
        [1.5, 1.5],
        [1.0, 0.4],
        [0.0, 0.0],
    ],
    dtype=float,
)

# Boundary headings are imposed by the user; intermediate headings are computed geometrically.
THETA_START = 2.7611
THETA_END = -2.7611

# Fill with len(WAYPOINTS) - 1 values. If left empty, times are computed from DEFAULT_SPEED.
SEGMENT_TIMES = [12.0, 10.1, 10.1, 9.0, 
                 9.0, 10.1, 10.1, 12.0]
EPS = 1e-9


# ---------------------------- Waypoint processing ---------------------------
# Return the Euclidean norm of a vector.
def vec_norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


# Normalize a vector while avoiding division by near-zero values.
def unit(v: np.ndarray) -> np.ndarray:
    n = vec_norm(v)
    return np.zeros(2) if n < EPS else v / n


# Return the 2D cross product used for curvature and turn direction.
def cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


# Validate that waypoints contain only X and Y coordinates.
def validate_waypoints(waypoints: np.ndarray) -> np.ndarray:
    waypoints = np.asarray(waypoints, dtype=float)
    if waypoints.ndim != 2 or waypoints.shape[1] != 2:
        raise ValueError("WAYPOINTS must have shape (n, 2) with columns [X, Y].")
    if waypoints.shape[0] < 2:
        raise ValueError("At least two waypoints are required.")
    return waypoints


# Validate or automatically compute one travelling time for each segment.
def get_segment_times_from_geometry(points: np.ndarray, manual_times=None) -> np.ndarray:
    if manual_times is not None:
        manual_times = np.asarray(manual_times, dtype=float)
        if manual_times.size:
            if manual_times.size != len(points) - 1:
                raise ValueError("SEGMENT_TIMES must contain len(WAYPOINTS) - 1 values.")
            if np.any(manual_times <= 0.0):
                raise ValueError("All SEGMENT_TIMES values must be strictly positive.")
            return manual_times

    times = []
    for p0, p1 in zip(points[:-1], points[1:]):
        segment_length = vec_norm(p1 - p0)
        segment_time = segment_length / max(DEFAULT_SPEED, EPS)
        times.append(TIME_SAFETY_FACTOR * max(segment_time, MIN_SEGMENT_TIME))
    return np.asarray(times, dtype=float)


# Estimate waypoint curvature from three neighboring points.
def discrete_curvature(points: np.ndarray, i: int) -> float:
    n = len(points)
    if i in (0, n - 1):
        if ZERO_ENDPOINT_STEERING or n < 3:
            return 0.0
        i = 1 if i == 0 else n - 2

    a_vec = points[i] - points[i - 1]
    b_vec = points[i + 1] - points[i]
    chord = points[i + 1] - points[i - 1]
    a, b, c = map(vec_norm, (a_vec, b_vec, chord))
    return 0.0 if min(a, b, c) < EPS else 2.0 * cross2(a_vec, b_vec) / (a * b * c)


# Compute the local tangent vector at an intermediate waypoint.
def tangent_at(points: np.ndarray, i: int) -> np.ndarray:
    if i == 0:
        return points[1] - points[0]
    if i == len(points) - 1:
        return points[-1] - points[-2]

    tangent = unit(points[i] - points[i - 1]) + unit(points[i + 1] - points[i])
    return points[i + 1] - points[i - 1] if vec_norm(tangent) < EPS else tangent


# Compute headings: start/end are user-imposed, intermediate ones use geometry.
def waypoint_headings(points: np.ndarray, theta_start: float, theta_end: float) -> np.ndarray:
    psi = np.zeros(len(points), dtype=float)
    psi[0] = float(theta_start)
    psi[-1] = float(theta_end)

    for i in range(1, len(points) - 1):
        tangent = tangent_at(points, i)
        psi[i] = np.arctan2(tangent[1], tangent[0])

    return np.unwrap(psi)


# Compute waypoint speeds from segment lengths and travelling times.
def waypoint_speeds_from_times(points: np.ndarray, segment_times: np.ndarray) -> np.ndarray:
    num_points = len(points)
    num_segments = num_points - 1

    segment_speeds = np.zeros(num_segments, dtype=float)
    for i in range(num_segments):
        segment_vector = points[i + 1] - points[i]
        segment_length = vec_norm(segment_vector)
        segment_time = max(segment_times[i], EPS)
        segment_speeds[i] = segment_length / segment_time

    waypoint_speeds = np.zeros(num_points, dtype=float)
    waypoint_speeds[0] = segment_speeds[0]
    waypoint_speeds[-1] = segment_speeds[-1]

    for i in range(1, num_points - 1):
        previous_segment_speed = segment_speeds[i - 1]
        next_segment_speed = segment_speeds[i]
        waypoint_speeds[i] = 0.5 * (previous_segment_speed + next_segment_speed)

    return waypoint_speeds


# Estimate longitudinal acceleration at each waypoint from speed changes.
def acceleration_from_waypoint_speeds(vx: np.ndarray, segment_times: np.ndarray) -> np.ndarray:
    ax = np.zeros_like(vx)

    if len(vx) > 1:
        segment_ax = (vx[1:] - vx[:-1]) / segment_times

        ax[0] = segment_ax[0]
        ax[-1] = segment_ax[-1]

        for i in range(1, len(vx) - 1):
            ax[i] = 0.5 * (segment_ax[i - 1] + segment_ax[i])

    return ax


# Add psi, vx, delta, and ax to build full single-track boundary waypoints.
def build_waypoints(
    waypoints: np.ndarray,
    theta_start: float = THETA_START,
    theta_end: float = THETA_END,
    segment_times=None,
) -> tuple[np.ndarray, np.ndarray]:
    points = validate_waypoints(waypoints)
    times = get_segment_times_from_geometry(points, segment_times)

    full_waypoints = np.zeros((len(points), 6), dtype=float)
    full_waypoints[:, :2] = points
    full_waypoints[:, 2] = waypoint_headings(points, theta_start, theta_end)
    full_waypoints[:, 3] = waypoint_speeds_from_times(points, times)

    for i in range(len(points)):
        kappa_i = discrete_curvature(points, i)
        full_waypoints[i, 4] = np.arctan(LWB * kappa_i)

    full_waypoints[:, 5] = acceleration_from_waypoint_speeds(full_waypoints[:, 3], times)
    return full_waypoints, times


# ------------------------- Bezier and flatness model -------------------------
# Evaluate all Bernstein polynomials of degree n.
def bernstein(n: int, s: np.ndarray) -> np.ndarray:
    return np.vstack([comb(n, i) * (1.0 - s) ** (n - i) * s**i for i in range(n + 1)])


# Convert a model waypoint into flat output derivatives.
def waypoint_derivatives(wp: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    X, Y, psi, vx, delta, ax = wp
    e_hat = np.array([np.cos(psi), np.sin(psi)])
    n_hat = np.array([-np.sin(psi), np.cos(psi)])

    z = np.array([X, Y])
    zd = vx * e_hat
    psi_dot = (vx / LWB) * np.tan(delta)
    zdd = ax * e_hat + vx * psi_dot * n_hat
    return z, zd, zdd


# Compute the six degree-5 Bezier control points from boundary conditions.
def bezier5_control_points(start: np.ndarray, finish: np.ndarray, T: float) -> np.ndarray:
    z0, zd0, zdd0 = waypoint_derivatives(start)
    zf, zdf, zddf = waypoint_derivatives(finish)

    P = np.zeros((2, 6), dtype=float)
    P[:, 0] = z0
    P[:, 1] = P[:, 0] + T * zd0 / 5.0
    P[:, 2] = 2.0 * P[:, 1] - P[:, 0] + T**2 * zdd0 / 20.0
    P[:, 5] = zf
    P[:, 4] = P[:, 5] - T * zdf / 5.0
    P[:, 3] = 2.0 * P[:, 4] - P[:, 5] + T**2 * zddf / 20.0
    return P


# Sample a degree-5 Bezier curve and its first two derivatives.
def evaluate_bezier5(P: np.ndarray, T: float, include_endpoint: bool) -> dict[str, np.ndarray]:
    t = np.arange(0.0, T, DT)
    if include_endpoint:
        if t.size == 0 or abs(t[-1] - T) > EPS:
            t = np.append(t, T)

    s = t / T
    d1 = P[:, 1:] - P[:, :-1]
    d2 = P[:, 2:] - 2.0 * P[:, 1:-1] + P[:, :-2]

    return {
        "t": t,
        "z": P @ bernstein(5, s),
        "zd": 5.0 / T * (d1 @ bernstein(4, s)),
        "zdd": 20.0 / T**2 * (d2 @ bernstein(3, s)),
    }


# Recover X, Y, psi, vx, delta, and ax from the flat output.
def references_from_flatness(traj: dict[str, np.ndarray]) -> None:
    z, zd, zdd = traj["z"], traj["zd"], traj["zdd"]

    Xd, Yd = zd
    Xdd, Ydd = zdd
    speed = np.hypot(Xd, Yd)
    speed_safe = np.maximum(speed, EPS)

    kappa_num = Xd * Ydd - Yd * Xdd
    kappa = kappa_num / np.maximum(speed_safe**3, EPS)
    ax = (Xd * Xdd + Yd * Ydd) / speed_safe

    traj.update(
        {
            "X": z[0],
            "Y": z[1],
            "psi": np.unwrap(np.arctan2(Yd, Xd)),
            "vx": speed,
            "delta": np.arctan(LWB * kappa),
            "ax": ax,
            "kappa": kappa,
        }
    )


# Join several Bezier segments into one continuous trajectory.
def generate_piecewise_bezier(waypoints: np.ndarray, segment_times: np.ndarray) -> tuple[dict[str, np.ndarray], list[np.ndarray]]:
    parts = {"t": [], "z": [], "zd": [], "zdd": [], "segment": []}
    control_points = []
    time_offset = 0.0

    for k, T in enumerate(segment_times):
        T = float(max(T, EPS))
        P = bezier5_control_points(waypoints[k], waypoints[k + 1], T)
        seg = evaluate_bezier5(P, T, include_endpoint=(k == len(segment_times) - 1))

        control_points.append(P)
        parts["t"].append(seg["t"] + time_offset)
        for key in ("z", "zd", "zdd"):
            parts[key].append(seg[key])
        parts["segment"].append(np.full_like(seg["t"], k, dtype=int))
        time_offset += float(T)

    traj = {
        "t": np.concatenate(parts["t"]),
        "z": np.hstack(parts["z"]),
        "zd": np.hstack(parts["zd"]),
        "zdd": np.hstack(parts["zdd"]),
        "segment": np.concatenate(parts["segment"]),
    }
    references_from_flatness(traj)
    return traj, control_points


# ------------------------------- Save outputs -------------------------------
# Save the trajectory, waypoints, and control points as CSV files.
def save_csv_outputs(traj: dict[str, np.ndarray], waypoints: np.ndarray, times: np.ndarray, cps: list[np.ndarray], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    np.savetxt(
        output_dir / "qcar2_single_track_flatness_bezier5_auto.csv",
        np.column_stack(
            (
                traj["t"], traj["segment"], traj["X"], traj["Y"], traj["psi"], traj["vx"],
                traj["delta"], traj["ax"], traj["zd"][0], traj["zd"][1], traj["zdd"][0], traj["zdd"][1],
                traj["kappa"],
            )
        ),
        delimiter=",",
        header="t,segment,X,Y,psi,vx,delta,ax,Xd,Yd,Xdd,Ydd,kappa",
        comments="",
    )

    cp_rows = [[k, i, P[0, i], P[1, i]] for k, P in enumerate(cps) for i in range(P.shape[1])]
    np.savetxt(output_dir / "bezier5_control_points.csv", cp_rows, delimiter=",", header="segment,control_point,X,Y", comments="")
    np.savetxt(output_dir / "computed_waypoints_full.csv", waypoints, delimiter=",", header="X,Y,psi,vx,delta,ax", comments="")
    np.savetxt(output_dir / "computed_segment_times.csv", np.column_stack((np.arange(len(times)), times)), delimiter=",", header="segment,T", comments="")


# Plot the reference trajectory for quick checks.
def save_plots(traj: dict[str, np.ndarray], waypoints: np.ndarray, cps: list[np.ndarray], output_dir: Path) -> None:
    # Finalize the current figure and save it.
    def finish(name: str, xlabel: str, ylabel: str, title: str) -> None:
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output_dir / name, dpi=180)
        plt.close()

    plt.figure()
    plt.plot(traj["X"], traj["Y"], label="Bezier trajectory")
    for k, P in enumerate(cps):
        plt.plot(P[0], P[1], "--o", label="Control polygon" if k == 0 else None)
    plt.plot(waypoints[:, 0], waypoints[:, 1], "ro", label="Waypoints")
    plt.axis("equal")
    plt.legend()
    finish("trajectory_xy.png", "X [m]", "Y [m]", "QCar2 single-track piecewise degree-5 Bezier trajectory")

    for key, ylabel, title, filename in [
        ("psi", "psi [rad]", "Heading angle", "heading.png"),
        ("vx", "vx [m/s]", "Longitudinal velocity", "speed.png"),
        ("delta", "delta [rad]", "Steering angle", "steering_angle.png"),
        ("ax", "ax [m/s^2]", "Longitudinal acceleration", "longitudinal_acceleration.png"),
    ]:
        plt.figure()
        plt.plot(traj["t"], traj[key])
        if key == "delta":
            plt.axhline(DELTA_MAX, linestyle="--")
            plt.axhline(-DELTA_MAX, linestyle="--")
        if key == "ax":
            plt.axhline(AX_MAX, linestyle="--")
            plt.axhline(-AX_MAX, linestyle="--")
        finish(filename, "t [s]", ylabel, title)


# Save all trajectory CSV files and figures.
def save_outputs(
    traj: dict[str, np.ndarray],
    waypoints: np.ndarray,
    times: np.ndarray,
    cps: list[np.ndarray],
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
    generate_plots: bool = True,
) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    save_csv_outputs(traj, waypoints, times, cps, output_dir)
    if generate_plots:
        save_plots(traj, waypoints, cps, output_dir)

    print("Generated QCar2 single-track piecewise degree-5 Bezier trajectory")
    print(f"Output folder        : {output_dir}")
    print(f"Waypoints / segments: {len(waypoints)} / {len(times)}")
    print(f"Total time           : {traj['t'][-1]:.4f} s")
    print(f"Max vx               : {np.max(np.abs(traj['vx'])):.4f} m/s")
    print(f"Max |delta|          : {np.max(np.abs(traj['delta'])):.4f} rad")
    print(f"Max |ax|             : {np.max(np.abs(traj['ax'])):.4f} m/s^2")

    if np.max(np.abs(waypoints[:, 4])) > DELTA_MAX:
        print("WARNING: waypoint steering angle limit exceeded. Adjust X,Y waypoints.")
    if np.max(np.abs(traj["delta"])) > DELTA_MAX:
        print("WARNING: trajectory steering angle limit exceeded. Adjust X,Y waypoints or headings.")
    if np.max(np.abs(traj["ax"])) > AX_MAX:
        print("WARNING: trajectory longitudinal acceleration limit exceeded. Increase segment times or smooth waypoints.")


# Generate the complete reference trajectory from waypoint geometry.
def generate_trajectory(
    waypoints: np.ndarray = WAYPOINTS,
    theta_start: float = THETA_START,
    theta_end: float = THETA_END,
    segment_times=None,
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
    generate_plots: bool = True,
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, list[np.ndarray]]:
    if segment_times is None:
        segment_times = SEGMENT_TIMES
    full_waypoints, times = build_waypoints(waypoints, theta_start, theta_end, segment_times)
    traj, control_points = generate_piecewise_bezier(full_waypoints, times)
    save_outputs(traj, full_waypoints, times, control_points, output_dir=output_dir, generate_plots=generate_plots)
    return traj, full_waypoints, times, control_points


# Allow this file to be run directly to generate the trajectory.
def main() -> None:
    generate_trajectory(WAYPOINTS, THETA_START, THETA_END, SEGMENT_TIMES, DEFAULT_OUTPUT_DIR, generate_plots=True)


if __name__ == "__main__":
    main()
