"""Matplotlib 3D animation for simulated disc flights."""

import math

import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .aerodynamics import rotate_body_to_world
from .field import FieldGeometry
from .flight import DiscProfile
from .simulation import FlightTrajectory


def _interpolate_quaternion(first: np.ndarray, second: np.ndarray, amount: float) -> np.ndarray:
    first = first / np.linalg.norm(first)
    second = second / np.linalg.norm(second)
    dot = float(np.dot(first, second))
    if dot < 0:
        second = -second
        dot = -dot
    if dot > 0.9995:
        result = first + amount * (second - first)
        return result / np.linalg.norm(result)
    angle = math.acos(min(1.0, dot))
    return (
        math.sin((1.0 - amount) * angle) / math.sin(angle) * first
        + math.sin(amount * angle) / math.sin(angle) * second
    )


def animate_flight(
    trajectory: FlightTrajectory,
    disc: DiscProfile,
    field: FieldGeometry = FieldGeometry(),
    frames_per_second: int = 30,
    trail_duration_s: float | None = None,
) -> FuncAnimation:
    """Create a 3D animation of a trajectory without opening a UI window.

    Call ``matplotlib.pyplot.show()`` to display the returned animation. The
    animation uses the trajectory's recorded attitude to draw the disc plane
    and linearly interpolates position and quaternion attitude between samples.
    """
    if not trajectory.samples:
        raise ValueError("trajectory must contain at least one sample")
    if isinstance(frames_per_second, bool) or not isinstance(frames_per_second, int):
        raise ValueError("frames_per_second must be a positive integer")
    if frames_per_second <= 0:
        raise ValueError("frames_per_second must be a positive integer")
    if trail_duration_s is not None and (
        isinstance(trail_duration_s, bool)
        or not isinstance(trail_duration_s, (int, float))
        or not math.isfinite(trail_duration_s)
        or trail_duration_s <= 0
    ):
        raise ValueError("trail_duration_s must be a finite positive number or None")

    import matplotlib.pyplot as plt

    times = np.asarray([sample.elapsed_s for sample in trajectory.samples], dtype=float)
    positions = np.asarray([sample.position_m for sample in trajectory.samples], dtype=float)
    orientations = np.asarray(
        [sample.orientation_wxyz for sample in trajectory.samples],
        dtype=float,
    )
    valid_samples = (
        np.all(np.isfinite(times))
        and np.all(np.isfinite(positions))
        and np.all(np.isfinite(orientations))
        and np.all(np.linalg.norm(orientations, axis=1) > 0)
        and (len(times) == 1 or np.all(np.diff(times) > 0))
    )
    if not valid_samples:
        raise ValueError("trajectory samples must have finite, ordered states")

    start_time = float(times[0])
    end_time = float(times[-1])
    frame_count = max(1, int(math.ceil((end_time - start_time) * frames_per_second)) + 1)
    frame_times = np.linspace(start_time, end_time, frame_count)

    figure = plt.figure(figsize=(12, 7))
    axes = figure.add_subplot(111, projection="3d")
    axes.set_title("Disc flight")
    axes.set_xlabel("Field length x (m)")
    axes.set_ylabel("Field width y (m)")
    axes.set_zlabel("Height z (m)")
    axes.view_init(elev=28, azim=-58)

    end_a = field.end_zone_depth_m
    end_b = field.length_m - field.end_zone_depth_m
    field_regions = [
        [
            (0.0, 0.0, 0.0),
            (end_a, 0.0, 0.0),
            (end_a, field.width_m, 0.0),
            (0.0, field.width_m, 0.0),
        ],
        [
            (end_a, 0.0, 0.0),
            (end_b, 0.0, 0.0),
            (end_b, field.width_m, 0.0),
            (end_a, field.width_m, 0.0),
        ],
        [
            (end_b, 0.0, 0.0),
            (field.length_m, 0.0, 0.0),
            (field.length_m, field.width_m, 0.0),
            (end_b, field.width_m, 0.0),
        ],
    ]
    axes.add_collection3d(
        Poly3DCollection(
            field_regions,
            facecolors=("#668b5a", "#78a96b", "#668b5a"),
            edgecolors="#f6f4e8",
            linewidths=1.1,
            alpha=0.65,
        )
    )
    for x in (end_a, end_b):
        axes.plot(
            (x, x),
            (0.0, field.width_m),
            (0.015, 0.015),
            color="#ffffff",
            linewidth=1.5,
        )
    axes.plot(
        (0.0, field.length_m, field.length_m, 0.0, 0.0),
        (0.0, 0.0, field.width_m, field.width_m, 0.0),
        (0.02, 0.02, 0.02, 0.02, 0.02),
        color="#ffffff",
        linewidth=1.5,
    )

    axes.plot(
        positions[:, 0],
        positions[:, 1],
        positions[:, 2],
        color="#343a40",
        alpha=0.28,
        linewidth=1.2,
        linestyle="--",
        label="Full trajectory",
    )
    trail, = axes.plot([], [], [], color="#e4572e", linewidth=2.5, label="Flight trail")
    rim, = axes.plot([], [], [], color="#165d9c", linewidth=2.0)
    diameter_line, = axes.plot([], [], [], color="#ffffff", linewidth=1.0)
    marker, = axes.plot([], [], [], marker="o", color="#e4572e", markersize=4)
    time_label = axes.text2D(0.02, 0.96, "", transform=axes.transAxes)
    axes.legend(loc="upper right")

    x_limits = (
        min(0.0, float(np.min(positions[:, 0]))),
        max(field.length_m, float(np.max(positions[:, 0]))),
    )
    y_limits = (
        min(0.0, float(np.min(positions[:, 1]))),
        max(field.width_m, float(np.max(positions[:, 1]))),
    )
    z_min = min(0.0, float(np.min(positions[:, 2])))
    z_max = max(1.0, float(np.max(positions[:, 2])) * 1.12)
    axes.set_xlim(*x_limits)
    axes.set_ylim(*y_limits)
    axes.set_zlim(z_min, z_max)
    axes.set_box_aspect(
        (
            max(1.0, x_limits[1] - x_limits[0]),
            max(1.0, y_limits[1] - y_limits[0]),
            max(1.0, z_max - z_min),
        )
    )

    radius = disc.diameter_m / 2
    local_angles = np.linspace(0.0, 2.0 * math.pi, 49)
    local_rim = np.column_stack(
        (
            radius * np.cos(local_angles),
            radius * np.sin(local_angles),
            np.zeros_like(local_angles),
        )
    )
    local_diameter = np.asarray(
        ((-radius, 0.0, 0.0), (radius, 0.0, 0.0)),
        dtype=float,
    )

    def state_at(time_s: float) -> tuple[np.ndarray, np.ndarray]:
        if len(times) == 1 or time_s <= times[0]:
            return positions[0], orientations[0] / np.linalg.norm(orientations[0])
        if time_s >= times[-1]:
            return positions[-1], orientations[-1] / np.linalg.norm(orientations[-1])
        upper = int(np.searchsorted(times, time_s, side="right"))
        lower = upper - 1
        amount = (time_s - times[lower]) / (times[upper] - times[lower])
        position = positions[lower] + amount * (positions[upper] - positions[lower])
        orientation = _interpolate_quaternion(
            orientations[lower],
            orientations[upper],
            amount,
        )
        return position, orientation

    def update(frame: int):
        time_s = float(frame_times[frame])
        position, orientation = state_at(time_s)
        trail_start = (
            start_time
            if trail_duration_s is None
            else max(start_time, time_s - trail_duration_s)
        )
        visible_times = frame_times[(frame_times >= trail_start) & (frame_times <= time_s)]
        visible_positions = np.asarray([state_at(value)[0] for value in visible_times])
        if len(visible_positions) == 0:
            visible_positions = position.reshape(1, 3)
        trail.set_data_3d(
            visible_positions[:, 0],
            visible_positions[:, 1],
            visible_positions[:, 2],
        )

        world_rim = np.asarray(
            [
                rotate_body_to_world(tuple(orientation), tuple(vertex))
                for vertex in local_rim
            ]
        ) + position
        rim.set_data_3d(world_rim[:, 0], world_rim[:, 1], world_rim[:, 2])
        world_diameter = np.asarray(
            [
                rotate_body_to_world(tuple(orientation), tuple(vertex))
                for vertex in local_diameter
            ]
        ) + position
        diameter_line.set_data_3d(
            world_diameter[:, 0],
            world_diameter[:, 1],
            world_diameter[:, 2],
        )
        marker.set_data_3d((position[0],), (position[1],), (position[2],))
        time_label.set_text(f"t = {time_s:.2f} s")
        return trail, rim, diameter_line, marker, time_label

    return FuncAnimation(
        figure,
        update,
        frames=frame_count,
        interval=1000 / frames_per_second,
        repeat=False,
        blit=False,
        cache_frame_data=False,
    )
