#!/usr/bin/env python3
"""Generate a QCar2 piecewise degree-7 Bezier trajectory from waypoint geometry."""

from __future__ import annotations

from pathlib import Path
from math import comb

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------- User parameters -----------------------------
L = 0.25725              # wheelbase [m]
DT = 0.02                # sample time [s]
VARPHI_MAX = 0.5236      # steering angle limit [rad]
OMEGA_S_MAX = 2.0        # steering angular velocity limit [rad/s]

ZERO_ENDPOINT_STEERING = True

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[2] / "data" / "reference"

# Edit this list in normal use: only Cartesian positions [x, y].
# Boundary headings and segment times are entered separately below.
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

# Prescribed heading at the first and last waypoint [rad], as in Section 4.1.
THETA_START = 2.7611
THETA_END = -2.7611

# Prescribed travelling time for each segment: len(SEGMENT_TIMES) must be len(WAYPOINTS) - 1.
SEGMENT_TIMES = [12.0, 10.1, 10.1, 9.0, 
                 9.0, 10.1, 10.1, 12.0]
EPS = 1e-9


# ---------------------------- Waypoint processing ---------------------------
def vec_norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


# Normalize a 2D vector when possible.
# A near-zero vector is kept safe by returning [0, 0].
def unit(v: np.ndarray) -> np.ndarray:
    n = vec_norm(v)
    return np.zeros(2) if n < EPS else v / n

# det(a,b)
def cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


# Validate the waypoint array shape before trajectory generation.
# Section 4.1 uses only Cartesian waypoint positions [x, y].
def validate_waypoints(waypoints: np.ndarray) -> np.ndarray:
    waypoints = np.asarray(waypoints, dtype=float)
    if waypoints.ndim != 2 or waypoints.shape[1] != 2:
        raise ValueError("WAYPOINTS must have shape (N, 2): each waypoint is [x, y].")
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required.")
    return waypoints


# Estimate local signed curvature from three consecutive waypoints.
# This follows the geometric curvature formula used before Bezier generation.
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


# Estimate the local tangent direction at a waypoint.
# Interior points use the sum of incoming and outgoing unit directions.
def tangent_at(points: np.ndarray, i: int) -> np.ndarray:
    if i == 0:
        return points[1] - points[0]
    if i == len(points) - 1:
        return points[-1] - points[-2]

    tangent = unit(points[i] - points[i - 1]) + unit(points[i + 1] - points[i])
    return points[i + 1] - points[i - 1] if vec_norm(tangent) < EPS else tangent


# Convert [x, y] waypoints into full states [x, y, theta, varphi, v, omega_s].
# Interior values are computed from geometry; endpoint headings are imposed by the user.
def build_waypoints(waypoints_input: np.ndarray, manual_times=None) -> tuple[np.ndarray, np.ndarray]:
    points = validate_waypoints(waypoints_input)

    waypoints = np.zeros((len(points), 6), dtype=float)
    waypoints[:, :2] = points

    segment_times = get_segment_times(points, manual_times)
    waypoints[:, 4] = waypoint_speeds_from_times(points, segment_times)

    for i, speed in enumerate(waypoints[:, 4]):
        tangent = tangent_at(points, i)
        sigma = 1.0 if speed >= 0.0 else -1.0
        motion_heading = np.arctan2(tangent[1], tangent[0])
        waypoints[i, 2] = motion_heading if sigma > 0.0 else motion_heading + np.pi
        waypoints[i, 3] = np.arctan(sigma * L * discrete_curvature(points, i))

    if THETA_START is not None:
        waypoints[0, 2] = float(THETA_START)
    if THETA_END is not None:
        waypoints[-1, 2] = float(THETA_END)

    waypoints[:, 2] = np.unwrap(waypoints[:, 2])
    waypoints[:, 5] = steering_velocity(waypoints[:, 3], segment_times)
    return waypoints, segment_times


# Return the prescribed segment durations entered by the user.
# Section 4.1 requires one travelling time T_i for each path segment.
def get_segment_times(points: np.ndarray, manual_times=None) -> np.ndarray:
    if manual_times is None:
        manual_times = SEGMENT_TIMES

    manual_times = np.asarray(manual_times, dtype=float)
    if len(manual_times) != len(points) - 1:
        raise ValueError(
            "SEGMENT_TIMES must contain exactly len(WAYPOINTS) - 1 values. "
            "Fill SEGMENT_TIMES = [...] with the prescribed travelling time of each segment."
        )
    if np.any(manual_times <= 0.0):
        raise ValueError("All SEGMENT_TIMES values must be strictly positive.")
    return manual_times


# Compute waypoint velocities from adjacent segment average speeds.
# This implements the velocity rule from Section 4.1 of the PDF.
def waypoint_speeds_from_times(points: np.ndarray, segment_times: np.ndarray) -> np.ndarray:
    num_points = len(points)
    num_segments = num_points - 1

    segment_speeds = np.zeros(num_segments, dtype=float)

    # Step 1: compute the average speed of each segment.
    for i in range(num_segments):
        segment_vector = points[i + 1] - points[i]
        segment_length = vec_norm(segment_vector)
        segment_time = max(segment_times[i], EPS)

        segment_speeds[i] = segment_length / segment_time

    waypoint_speeds = np.zeros(num_points, dtype=float)

    # Step 2: assign speed at each waypoint from adjacent segment speeds.
    waypoint_speeds[0] = segment_speeds[0]
    waypoint_speeds[-1] = segment_speeds[-1]

    for i in range(1, num_points - 1):
        previous_segment_speed = segment_speeds[i - 1]
        next_segment_speed = segment_speeds[i]

        waypoint_speeds[i] = 0.5 * (
            previous_segment_speed + next_segment_speed
        )

    return waypoint_speeds


# Approximate steering angular velocity at each waypoint.
# Endpoints use one-sided differences; interior points use centered differences.
def steering_velocity(varphi: np.ndarray, segment_times: np.ndarray) -> np.ndarray:
    omega = np.zeros_like(varphi)
    if len(varphi) > 2:
        omega[1:-1] = (varphi[2:] - varphi[:-2]) / (segment_times[:-1] + segment_times[1:])
    if len(varphi) > 1:
        omega[0] = (varphi[1] - varphi[0]) / segment_times[0]
        omega[-1] = (varphi[-1] - varphi[-2]) / segment_times[-1]
    return omega


# ------------------------- Bezier and flatness model -------------------------
def bernstein(n: int, s: np.ndarray) -> np.ndarray:
    return np.vstack([comb(n, i) * (1.0 - s) ** (n - i) * s**i for i in range(n + 1)])


# Convert one full waypoint into flat-output derivatives.
# These derivatives define the degree-7 Bezier boundary conditions.
def waypoint_derivatives(wp: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x, y, theta, varphi, v, omega_s = wp
    e_hat = np.array([np.cos(theta), np.sin(theta)])
    n_hat = np.array([-np.sin(theta), np.cos(theta)])

    z = np.array([x, y])
    zd = v * e_hat
    theta_dot = (v / L) * np.tan(varphi)
    theta_ddot = (v / L) * omega_s / np.cos(varphi) ** 2
    zdd = v * theta_dot * n_hat
    zddd = -v * theta_dot**2 * e_hat + v * theta_ddot * n_hat
    return z, zd, zdd, zddd


# Build the eight control points of one degree-7 Bezier segment.
# Boundary derivatives up to third order are imposed at both ends.
def bezier7_control_points(start: np.ndarray, finish: np.ndarray, T: float) -> np.ndarray:
    z0, zd0, zdd0, zddd0 = waypoint_derivatives(start)
    zf, zdf, zddf, zdddf = waypoint_derivatives(finish)

    P = np.zeros((2, 8), dtype=float)
    P[:, 0] = z0
    P[:, 1] = P[:, 0] + T * zd0 / 7.0
    P[:, 2] = 2.0 * P[:, 1] - P[:, 0] + T**2 * zdd0 / 42.0
    P[:, 3] = 3.0 * P[:, 2] - 3.0 * P[:, 1] + P[:, 0] + T**3 * zddd0 / 210.0
    P[:, 7] = zf
    P[:, 6] = P[:, 7] - T * zdf / 7.0
    P[:, 5] = T**2 * zddf / 42.0 - P[:, 7] + 2.0 * P[:, 6]
    P[:, 4] = P[:, 7] - 3.0 * P[:, 6] + 3.0 * P[:, 5] - T**3 * zdddf / 210.0
    return P


# Sample a Bezier segment and its first three derivatives.
# Time is normalized internally from t in [0, T] to s in [0, 1].
def evaluate_bezier7(P: np.ndarray, T: float, include_endpoint: bool) -> dict[str, np.ndarray]:
    t = np.arange(0.0, T + (EPS if include_endpoint else 0.0), DT)
    if t.size == 0:
        t = np.array([0.0])

    s = t / T
    d1 = P[:, 1:] - P[:, :-1]
    d2 = P[:, 2:] - 2.0 * P[:, 1:-1] + P[:, :-2]
    d3 = P[:, 3:] - 3.0 * P[:, 2:-1] + 3.0 * P[:, 1:-2] - P[:, :-3]

    return {
        "t": t,
        "z": P @ bernstein(7, s),
        "zd": 7.0 / T * (d1 @ bernstein(6, s)),
        "zdd": 42.0 / T**2 * (d2 @ bernstein(5, s)),
        "zddd": 210.0 / T**3 * (d3 @ bernstein(4, s)),
    }


# Recover QCar2 states and inputs from flat-output derivatives.
# The generated trajectory becomes x, y, theta, varphi, v, and omega_s references.
def references_from_flatness(traj: dict[str, np.ndarray]) -> None:
    z, zd, zdd, zddd = traj["z"], traj["zd"], traj["zdd"], traj["zddd"]
    sigma = np.where(traj["sigma"] >= 0.0, 1.0, -1.0)

    xd, yd = zd
    xdd, ydd = zdd
    xddd, yddd = zddd
    speed = np.hypot(xd, yd)
    speed_safe = np.maximum(speed, EPS)

    motion_heading = np.arctan2(yd, xd)
    kappa_num = xd * ydd - yd * xdd
    kappa = kappa_num / np.maximum(speed_safe**3, EPS)
    kappa_num_dot = xd * yddd - yd * xddd
    speed3_dot = 3.0 * speed_safe * (xd * xdd + yd * ydd)
    kappa_dot = (kappa_num_dot * speed_safe**3 - kappa_num * speed3_dot) / np.maximum(speed_safe**6, EPS)

    traj.update(
        {
            "x": z[0],
            "y": z[1],
            "theta": np.unwrap(np.where(sigma > 0.0, motion_heading, motion_heading + np.pi)),
            "varphi": np.arctan(sigma * L * kappa),
            "v": sigma * speed,
            "omega_s": sigma * L * kappa_dot / (1.0 + (sigma * L * kappa) ** 2),
            "kappa": kappa,
            "sigma": sigma,
        }
    )


# Choose forward or reverse motion direction for a segment.
def segment_direction(w0: np.ndarray, w1: np.ndarray, k: int) -> float:
    v0, v1 = w0[4], w1[4]
    if abs(v0) > EPS:
        return 1.0 if v0 > 0.0 else -1.0
    if abs(v1) > EPS:
        return 1.0 if v1 > 0.0 else -1.0
    return 1.0


# Generate all Bezier segments and concatenate them into one trajectory.
# Segment indices and time offsets are preserved in the output.
def generate_piecewise_bezier(waypoints: np.ndarray, segment_times: np.ndarray) -> tuple[dict[str, np.ndarray], list[np.ndarray]]:
    parts = {"t": [], "z": [], "zd": [], "zdd": [], "zddd": [], "sigma": [], "segment": []}
    control_points = []
    time_offset = 0.0

    for k, T in enumerate(segment_times):
        T = float(max(T, EPS))
        P = bezier7_control_points(waypoints[k], waypoints[k + 1], T)
        seg = evaluate_bezier7(P, T, include_endpoint=(k == len(segment_times) - 1))
        sigma = segment_direction(waypoints[k], waypoints[k + 1], k)

        control_points.append(P)
        parts["t"].append(seg["t"] + time_offset)
        for key in ("z", "zd", "zdd", "zddd"):
            parts[key].append(seg[key])
        parts["sigma"].append(np.full_like(seg["t"], sigma, dtype=float))
        parts["segment"].append(np.full_like(seg["t"], k, dtype=int))
        time_offset += float(T)

    traj = {
        "t": np.concatenate(parts["t"]),
        "z": np.hstack(parts["z"]),
        "zd": np.hstack(parts["zd"]),
        "zdd": np.hstack(parts["zdd"]),
        "zddd": np.hstack(parts["zddd"]),
        "sigma": np.concatenate(parts["sigma"]),
        "segment": np.concatenate(parts["segment"]),
    }
    references_from_flatness(traj)
    return traj, control_points


# ------------------------------- Save outputs -------------------------------
def save_csv_outputs(traj: dict[str, np.ndarray], waypoints: np.ndarray, times: np.ndarray, cps: list[np.ndarray], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    np.savetxt(
        output_dir / "qcar2_flatness_bezier7_auto.csv",
        np.column_stack(
            (
                traj["t"], traj["segment"], traj["x"], traj["y"], traj["theta"], traj["varphi"],
                traj["v"], traj["omega_s"], traj["zd"][0], traj["zd"][1], traj["zdd"][0], traj["zdd"][1],
                traj["zddd"][0], traj["zddd"][1], traj["kappa"],
            )
        ),
        delimiter=",",
        header="t,segment,x,y,theta,varphi,v,omega_s,xd,yd,xdd,ydd,xddd,yddd,kappa",
        comments="",
    )

    cp_rows = [[k, i, P[0, i], P[1, i]] for k, P in enumerate(cps) for i in range(P.shape[1])]
    np.savetxt(output_dir / "bezier7_control_points.csv", cp_rows, delimiter=",", header="segment,control_point,x,y", comments="")
    np.savetxt(output_dir / "computed_waypoints_full.csv", waypoints, delimiter=",", header="x,y,theta,varphi,v,omega_s", comments="")
    np.savetxt(output_dir / "computed_segment_times.csv", np.column_stack((np.arange(len(times)), times)), delimiter=",", header="segment,T", comments="")


# Save diagnostic plots for trajectory, states, and inputs.
# Steering and steering-rate plots include the physical bounds.
def save_plots(traj: dict[str, np.ndarray], waypoints: np.ndarray, cps: list[np.ndarray], output_dir: Path) -> None:
    # Apply common plot formatting and write the current figure to disk.
    # The figure is closed afterward to avoid memory buildup.
    def finish(name: str, xlabel: str, ylabel: str, title: str) -> None:
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(output_dir / name, dpi=180)
        plt.close()

    plt.figure()
    plt.plot(traj["x"], traj["y"], label="Bezier trajectory")
    for k, P in enumerate(cps):
        plt.plot(P[0], P[1], "--o", label="Control polygon" if k == 0 else None)
    plt.plot(waypoints[:, 0], waypoints[:, 1], "ro", label="Waypoints")
    plt.axis("equal")
    plt.legend()
    finish("trajectory_xy.png", "x [m]", "y [m]", "QCar2 piecewise degree-7 Bezier trajectory")

    for key, ylabel, title, filename in [
        ("theta", "theta [rad]", "Heading angle", "heading.png"),
        ("v", "v [m/s]", "Longitudinal velocity", "speed.png"),
        ("varphi", "varphi [rad]", "Steering angle", "steering_angle.png"),
        ("omega_s", "omega_s [rad/s]", "Steering angular velocity", "steering_angular_velocity.png"),
    ]:
        plt.figure()
        plt.plot(traj["t"], traj[key])
        if key == "varphi":
            plt.axhline(VARPHI_MAX, linestyle="--")
            plt.axhline(-VARPHI_MAX, linestyle="--")
        if key == "omega_s":
            plt.axhline(OMEGA_S_MAX, linestyle="--")
            plt.axhline(-OMEGA_S_MAX, linestyle="--")
        finish(filename, "t [s]", ylabel, title)


# Write all trajectory outputs and print a short feasibility summary.
# Warnings are printed when steering or steering-rate limits are exceeded.
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

    print("Generated QCar2 piecewise degree-7 Bezier trajectory")
    print(f"Output folder      : {output_dir}")
    print(f"Waypoints / segments: {len(waypoints)} / {len(times)}")
    print(f"Total time         : {traj['t'][-1]:.4f} s")
    print(f"Max speed          : {np.max(np.abs(traj['v'])):.4f} m/s")
    print(f"Max |varphi|       : {np.max(np.abs(traj['varphi'])):.4f} rad")
    print(f"Max |omega_s|      : {np.max(np.abs(traj['omega_s'])):.4f} rad/s")

    if np.max(np.abs(waypoints[:, 3])) > VARPHI_MAX:
        print("WARNING: waypoint steering angle limit exceeded. Adjust x,y waypoints.")
    if np.max(np.abs(traj["varphi"])) > VARPHI_MAX:
        print("WARNING: trajectory steering angle limit exceeded. Adjust x,y waypoints.")
    if np.max(np.abs(traj["omega_s"])) > OMEGA_S_MAX:
        print("WARNING: trajectory steering angular velocity limit exceeded. Increase segment times or smooth waypoints.")


# High-level trajectory generation entry point used by scripts.
# It builds waypoints, generates Bezier segments, and saves outputs.
def generate_trajectory(
    waypoints: np.ndarray = WAYPOINTS,
    segment_times=None,
    output_dir: str | Path = DEFAULT_OUTPUT_DIR,
    generate_plots: bool = True,
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray, list[np.ndarray]]:
    full_waypoints, times = build_waypoints(waypoints, segment_times)
    traj, control_points = generate_piecewise_bezier(full_waypoints, times)
    save_outputs(traj, full_waypoints, times, control_points, output_dir=output_dir, generate_plots=generate_plots)
    return traj, full_waypoints, times, control_points


# Run this module as a script with the default parameters.
# Useful for regenerating reference files from the command line.
def main() -> None:
    generate_trajectory(WAYPOINTS, SEGMENT_TIMES, DEFAULT_OUTPUT_DIR, generate_plots=True)


if __name__ == "__main__":
    main()
