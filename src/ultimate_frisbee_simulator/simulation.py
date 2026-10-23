"""Coupled rigid-body integration for disc-flight trajectories."""

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np
from scipy.integrate import solve_ivp

from .aerodynamics import evaluate_aerodynamics, rotate_body_to_world
from .flight import DiscProfile, FlightEnvironment, FlightInitialState, SolverSettings


class FlightTerminationReason(Enum):
    GROUND_CONTACT = "ground_contact"
    MAXIMUM_DURATION = "maximum_duration"


@dataclass(frozen=True)
class TrajectorySample:
    """A sampled flight state in SI units."""

    elapsed_s: float
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]
    angular_velocity_body_rad_s: tuple[float, float, float]


@dataclass(frozen=True)
class FlightTrajectory:
    """Ordered trajectory samples and the condition that ended the flight."""

    samples: tuple[TrajectorySample, ...]
    termination_reason: FlightTerminationReason


def _normalized_quaternion(values: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm == 0:
        raise RuntimeError("flight integration produced an invalid orientation")
    return values / norm


def _quaternion_derivative(
    orientation_wxyz: np.ndarray, angular_velocity_body_rad_s: np.ndarray
) -> np.ndarray:
    w, x, y, z = orientation_wxyz
    wx, wy, wz = angular_velocity_body_rad_s
    return 0.5 * np.array(
        (
            -x * wx - y * wy - z * wz,
            w * wx + y * wz - z * wy,
            w * wy + z * wx - x * wz,
            w * wz + x * wy - y * wx,
        ),
        dtype=float,
    )


def simulate_flight(
    disc: DiscProfile,
    initial_state: FlightInitialState,
    environment: FlightEnvironment = FlightEnvironment(),
    settings: SolverSettings = SolverSettings(),
) -> FlightTrajectory:
    """Integrate a disc's coupled translational and rotational motion.

    The body frame uses x forward, y spanwise, and z along the disc normal.
    Gravity acts along negative world z. SciPy's RK45 solver is deterministic
    for fixed inputs, settings, and dependency versions.
    """
    if initial_state.position_m[2] < 0:
        raise ValueError("initial disc altitude must not be below ground")

    moments_of_inertia = np.asarray(disc.principal_moments_kg_m2, dtype=float)
    initial = np.asarray(
        (
            *initial_state.position_m,
            *initial_state.velocity_m_s,
            *initial_state.orientation_wxyz,
            *initial_state.angular_velocity_body_rad_s,
        ),
        dtype=float,
    )

    def derivative(_time_s: float, state: np.ndarray) -> np.ndarray:
        position = state[0:3]
        velocity = state[3:6]
        orientation = _normalized_quaternion(state[6:10])
        angular_velocity = state[10:13]
        wrench = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=tuple(velocity),
            orientation_wxyz=tuple(orientation),
            wind_velocity_m_s=environment.wind_velocity_m_s,
            air_density_kg_m3=environment.air_density_kg_m3,
        )
        force_world = np.asarray(
            rotate_body_to_world(tuple(orientation), wrench.force_body_n),
            dtype=float,
        )
        acceleration = force_world / disc.mass_kg + np.array(
            (0.0, 0.0, -environment.gravity_m_s2),
            dtype=float,
        )
        angular_momentum = moments_of_inertia * angular_velocity
        angular_acceleration = (
            np.asarray(wrench.moment_body_nm, dtype=float)
            - np.cross(angular_velocity, angular_momentum)
        ) / moments_of_inertia
        return np.concatenate(
            (
                velocity,
                acceleration,
                _quaternion_derivative(orientation, angular_velocity),
                angular_acceleration,
            )
        )

    def ground_contact(_time_s: float, state: np.ndarray) -> float:
        return float(state[2])

    ground_contact.terminal = True
    ground_contact.direction = -1

    solution = solve_ivp(
        derivative,
        (0.0, settings.maximum_duration_s),
        initial,
        method="RK45",
        rtol=settings.relative_tolerance,
        atol=settings.absolute_tolerance,
        max_step=settings.maximum_step_s,
        events=ground_contact,
    )
    if not solution.success:
        raise RuntimeError(f"flight integration failed: {solution.message}")
    if solution.t.size == 0:
        raise RuntimeError("flight integration returned no trajectory samples")

    termination_reason = (
        FlightTerminationReason.GROUND_CONTACT
        if solution.t_events[0].size
        else FlightTerminationReason.MAXIMUM_DURATION
    )
    samples = tuple(
        TrajectorySample(
            elapsed_s=float(time_s),
            position_m=tuple(float(value) for value in state[0:3]),
            velocity_m_s=tuple(float(value) for value in state[3:6]),
            orientation_wxyz=tuple(
                float(value) for value in _normalized_quaternion(state[6:10])
            ),
            angular_velocity_body_rad_s=tuple(float(value) for value in state[10:13]),
        )
        for time_s, state in zip(solution.t, solution.y.T)
    )
    return FlightTrajectory(samples, termination_reason)
