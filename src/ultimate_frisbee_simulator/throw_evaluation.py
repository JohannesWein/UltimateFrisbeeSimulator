"""Nominal release solving for tactical pass plans."""

from dataclasses import dataclass, replace
from enum import Enum
import math
from numbers import Integral

import numpy as np
from scipy.optimize import least_squares

from .flight import (
    DiscProfile,
    FlightEnvironment,
    FlightInitialState,
    SolverSettings,
)
from .simulation import (
    FlightTerminationReason,
    FlightTrajectory,
    simulate_flight,
)
from .throwing import PlannedPass, TargetPassRequest, ThrowProfile, _finite


@dataclass(frozen=True)
class AutomaticTimingSettings:
    """Finite ascending time grid and per-candidate optimization budget."""

    minimum_catch_time_s: float = 0.3
    maximum_catch_time_s: float = 3.0
    candidate_count: int = 10
    maximum_evaluations_per_candidate: int = 20

    def __post_init__(self) -> None:
        for name in ("minimum_catch_time_s", "maximum_catch_time_s"):
            value = _finite(getattr(self, name), name)
            if value <= 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        if self.maximum_catch_time_s <= self.minimum_catch_time_s:
            raise ValueError("maximum_catch_time_s must exceed minimum_catch_time_s")
        for name, minimum in (
            ("candidate_count", 2),
            ("maximum_evaluations_per_candidate", 1),
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")


class ReleaseFailure(Enum):
    OPTIMIZATION_FAILED = "optimization_failed"
    TARGET_OUT_OF_TOLERANCE = "target_out_of_tolerance"
    GROUND_BEFORE_CATCH = "ground_before_catch"
    CATCH_AFTER_SIMULATION_LIMIT = "catch_after_simulation_limit"


@dataclass(frozen=True)
class NominalReleaseResult:
    """A verified release or search failure; failure is not proof of impossibility."""

    launch_state: FlightInitialState | None
    trajectory: FlightTrajectory | None
    target_error_m: float | None
    failure: ReleaseFailure | None
    optimizer_evaluations: int

    @property
    def feasible(self) -> bool:
        return self.launch_state is not None and self.trajectory is not None

    @property
    def diagnostic(self) -> str:
        if self.feasible:
            return "Verified nominal release found."
        details = {
            ReleaseFailure.GROUND_BEFORE_CATCH:
                "The last candidate contacted the ground before the requested catch time.",
            ReleaseFailure.TARGET_OUT_OF_TOLERANCE:
                "The last candidate missed the target tolerance at the requested catch time.",
            ReleaseFailure.OPTIMIZATION_FAILED:
                "The optimizer exhausted its evaluation budget without a verified release.",
            ReleaseFailure.CATCH_AFTER_SIMULATION_LIMIT:
                "The requested catch time exceeds the configured simulation duration.",
        }
        if self.failure not in details:
            raise ValueError("failed nominal release is missing a failure reason")
        return (
            "No suitable nominal release found. "
            f"{details[self.failure]} "
            "This does not prove the pass is physically impossible."
        )


@dataclass(frozen=True)
class AutomaticPassResult:
    """A resolved plan or a failed bounded search with its last candidate."""

    planned_pass: PlannedPass | None
    release_result: NominalReleaseResult
    attempted_catch_times_s: tuple[float, ...]

    @property
    def feasible(self) -> bool:
        return self.planned_pass is not None and self.release_result.feasible

    @property
    def diagnostic(self) -> str:
        if self.feasible:
            return "Verified target-only pass found."
        return (
            "No suitable release found within the automatic timing search bounds and budget. "
            f"{self.release_result.diagnostic}"
        )


def plan_target_pass(
    request: TargetPassRequest,
    disc: DiscProfile,
    environment: FlightEnvironment = FlightEnvironment(),
    settings: SolverSettings = SolverSettings(),
    *,
    timing: AutomaticTimingSettings = AutomaticTimingSettings(),
) -> AutomaticPassResult:
    """Return the first verified release on an ascending, finite time grid.

    This finds the earliest successful examined time, not a global optimum.
    Integration errors propagate; an exhausted search is an explicit result.
    """
    if timing.maximum_catch_time_s > settings.maximum_duration_s:
        raise ValueError("automatic timing bounds exceed the configured simulation duration")
    attempted: list[float] = []
    for time_s in np.linspace(
        timing.minimum_catch_time_s,
        timing.maximum_catch_time_s,
        int(timing.candidate_count),
    ):
        candidate = request.at_catch_time(float(time_s))
        release = solve_nominal_release(
            candidate,
            request.throw_profile,
            disc,
            environment,
            settings,
            maximum_evaluations=timing.maximum_evaluations_per_candidate,
        )
        attempted.append(candidate.nominal_catch_time_s)
        if release.feasible:
            return AutomaticPassResult(candidate, release, tuple(attempted))
    return AutomaticPassResult(None, release, tuple(attempted))


def solve_nominal_release(
    planned_pass: PlannedPass,
    throw_profile: ThrowProfile,
    disc: DiscProfile,
    environment: FlightEnvironment = FlightEnvironment(),
    settings: SolverSettings = SolverSettings(),
    *,
    maximum_evaluations: int = 20,
) -> NominalReleaseResult:
    """Find a bounded release velocity that reaches the planned catch point.

    The solver uses speed, horizontal aim offset, and elevation as its free
    parameters. A returned solution is accepted only when an exact-time
    trajectory sample falls within the pass's configured target tolerance. Its
    returned trajectory ends at the planned catch time.
    """
    if (
        isinstance(maximum_evaluations, bool)
        or not isinstance(maximum_evaluations, Integral)
        or maximum_evaluations < 1
    ):
        raise ValueError("maximum_evaluations must be a positive integer")
    if planned_pass.nominal_catch_time_s > settings.maximum_duration_s:
        return NominalReleaseResult(
            None,
            None,
            None,
            ReleaseFailure.CATCH_AFTER_SIMULATION_LIMIT,
            0,
        )

    origin = np.asarray(planned_pass.throw_origin_m, dtype=float)
    target = np.asarray(planned_pass.catch_target_m, dtype=float)
    horizontal_delta = target[:2] - origin[:2]
    horizontal_distance = float(np.linalg.norm(horizontal_delta))
    target_heading = math.atan2(float(horizontal_delta[1]), float(horizontal_delta[0]))
    catch_time = planned_pass.nominal_catch_time_s
    optimization_settings = replace(
        settings,
        maximum_duration_s=catch_time,
        relative_tolerance=max(settings.relative_tolerance, 1e-6),
        absolute_tolerance=max(settings.absolute_tolerance, 1e-8),
    )
    verification_settings = replace(
        settings,
        maximum_duration_s=catch_time,
    )

    minimum = np.asarray(
        (
            throw_profile.minimum_speed_m_s,
            -throw_profile.maximum_aim_offset_rad,
            throw_profile.minimum_elevation_rad,
        )
    )
    maximum = np.asarray(
        (
            throw_profile.maximum_speed_m_s,
            throw_profile.maximum_aim_offset_rad,
            throw_profile.maximum_elevation_rad,
        )
    )
    vertical_speed = (
        target[2] - origin[2] + 0.5 * environment.gravity_m_s2 * catch_time**2
    ) / catch_time
    horizontal_speed = max(horizontal_distance / catch_time, 0.1)
    initial_guess = np.asarray(
        (
            math.hypot(horizontal_speed, vertical_speed),
            0.0,
            math.atan2(vertical_speed, horizontal_speed),
        )
    )
    margin = np.maximum((maximum - minimum) * 1e-8, 1e-10)
    initial_guess = np.clip(initial_guess, minimum + margin, maximum - margin)

    def make_launch(parameters: np.ndarray) -> FlightInitialState:
        speed, aim_offset, elevation = (float(value) for value in parameters)
        heading = target_heading + aim_offset
        horizontal_speed_at_release = speed * math.cos(elevation)
        return FlightInitialState(
            position_m=planned_pass.throw_origin_m,
            velocity_m_s=(
                horizontal_speed_at_release * math.cos(heading),
                horizontal_speed_at_release * math.sin(heading),
                speed * math.sin(elevation),
            ),
            orientation_wxyz=throw_profile.orientation_wxyz,
            angular_velocity_body_rad_s=(0.0, 0.0, throw_profile.nominal_spin_rad_s),
        )

    def evaluate(
        parameters: np.ndarray,
        flight_settings: SolverSettings,
    ) -> tuple[np.ndarray, FlightTrajectory, FlightInitialState]:
        launch = make_launch(parameters)
        trajectory = simulate_flight(
            disc,
            launch,
            environment,
            flight_settings,
            sample_times_s=(catch_time,),
        )
        catch_sample = next(
            (
                sample
                for sample in trajectory.samples
                if sample.elapsed_s == catch_time
            ),
            None,
        )
        endpoint = (
            np.asarray(catch_sample.position_m, dtype=float)
            if catch_sample is not None
            else np.asarray(trajectory.samples[-1].position_m, dtype=float)
        )
        return endpoint - target, trajectory, launch

    def residual(parameters: np.ndarray) -> np.ndarray:
        error, _, _ = evaluate(parameters, optimization_settings)
        return error

    optimization = least_squares(
        residual,
        initial_guess,
        bounds=(minimum, maximum),
        max_nfev=int(maximum_evaluations),
        diff_step=1e-2,
        x_scale=np.asarray((10.0, 0.1, 0.1)),
    )
    error, trajectory, launch = evaluate(optimization.x, verification_settings)
    error_distance = float(np.linalg.norm(error))
    exact_catch_sample = any(
        sample.elapsed_s == catch_time for sample in trajectory.samples
    )
    if (
        exact_catch_sample
        and error_distance <= planned_pass.target_tolerance_m
    ):
        return NominalReleaseResult(
            launch,
            trajectory,
            error_distance,
            None,
            int(optimization.nfev),
        )

    if (
        not exact_catch_sample
        and trajectory.termination_reason is FlightTerminationReason.GROUND_CONTACT
    ):
        failure = ReleaseFailure.GROUND_BEFORE_CATCH
    elif not optimization.success and optimization.nfev >= maximum_evaluations:
        failure = ReleaseFailure.OPTIMIZATION_FAILED
    else:
        failure = ReleaseFailure.TARGET_OUT_OF_TOLERANCE
    return NominalReleaseResult(
        None,
        trajectory,
        error_distance if exact_catch_sample else None,
        failure,
        int(optimization.nfev),
    )
