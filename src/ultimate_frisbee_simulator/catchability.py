"""Seeded flight-level catchability estimates for planned passes."""

from dataclasses import dataclass, replace
import math
from numbers import Integral, Real

import numpy as np

from .flight import DiscProfile, FlightEnvironment, SolverSettings
from .simulation import simulate_flight
from .throw_evaluation import NominalReleaseResult
from .throwing import PlannedPass, ThrowProfile
from .throw_variation import ThrowVariationProfile, sample_throw_execution


class ThrowEvaluationError(RuntimeError):
    """A specific Monte Carlo trial failed to produce a valid flight."""


@dataclass(frozen=True)
class CatchabilityResult:
    """Trial outcomes and provenance for a flight-level catchability estimate."""

    trial_successes: tuple[bool, ...]
    flight_model_provenance: str
    throw_variation_provenance: str

    def __post_init__(self) -> None:
        successes = tuple(self.trial_successes)
        if not successes or any(not isinstance(success, bool) for success in successes):
            raise ValueError("trial_successes must contain at least one boolean result")
        object.__setattr__(self, "trial_successes", successes)
        for name in ("flight_model_provenance", "throw_variation_provenance"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must not be empty")

    @property
    def successful_trial_count(self) -> int:
        return sum(self.trial_successes)

    @property
    def total_trial_count(self) -> int:
        return len(self.trial_successes)

    @property
    def catchability_fraction(self) -> float:
        return self.successful_trial_count / self.total_trial_count

    @property
    def interpretation(self) -> str:
        return "flight-level catchability; not full pass-completion probability"


def _positive_finite(value: Real, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return result


def evaluate_flight_catchability(
    planned_pass: PlannedPass,
    nominal_release: NominalReleaseResult,
    throw_profile: ThrowProfile,
    variation_profile: ThrowVariationProfile,
    disc: DiscProfile,
    generator: np.random.Generator,
    *,
    trial_count: int = 100,
    time_step_s: float = 0.02,
    environment: FlightEnvironment = FlightEnvironment(),
    settings: SolverSettings = SolverSettings(),
) -> CatchabilityResult:
    """Estimate catch-window reachability using reproducible physical throws.

    Each trial succeeds when a simulated disc position enters the configured
    3D catch radius during the nominal time window. A failed flight integration
    is raised with its trial number rather than counted as an unsuccessful pass.
    """
    if (
        isinstance(trial_count, bool)
        or not isinstance(trial_count, Integral)
        or trial_count < 1
    ):
        raise ValueError("trial_count must be a positive integer")
    step = _positive_finite(time_step_s, "time_step_s")
    if not isinstance(generator, np.random.Generator):
        raise TypeError("generator must be a numpy.random.Generator")
    if not nominal_release.feasible or nominal_release.launch_state is None:
        raise ValueError("nominal_release must contain a feasible launch state")
    if throw_profile.throw_type is not variation_profile.throw_type:
        raise ValueError("throw and variation profiles must have the same throw_type")
    if planned_pass.nominal_catch_time_s > settings.maximum_duration_s:
        raise ValueError("nominal catch time exceeds the configured simulation duration")

    time_start_s = max(
        0.0,
        planned_pass.nominal_catch_time_s - planned_pass.catch_time_tolerance_s,
    )
    time_end_s = min(
        settings.maximum_duration_s,
        planned_pass.nominal_catch_time_s + planned_pass.catch_time_tolerance_s,
    )
    sample_count = max(2, math.ceil((time_end_s - time_start_s) / step) + 1)
    sample_times_s = tuple(
        float(value)
        for value in np.linspace(time_start_s, time_end_s, sample_count)
    )
    flight_settings = replace(settings, maximum_duration_s=time_end_s)
    target = np.asarray(planned_pass.catch_target_m, dtype=float)
    successes: list[bool] = []

    for trial_index in range(int(trial_count)):
        try:
            launch = sample_throw_execution(
                planned_pass,
                nominal_release,
                throw_profile,
                variation_profile,
                generator,
            )
            trajectory = simulate_flight(
                disc,
                launch,
                environment,
                flight_settings,
                sample_times_s=sample_times_s,
            )
        except (RuntimeError, ValueError) as error:
            raise ThrowEvaluationError(
                f"throw trial {trial_index + 1} failed: {error}"
            ) from error

        success = any(
            time_start_s <= sample.elapsed_s <= time_end_s
            and math.dist(sample.position_m, target) <= planned_pass.catch_radius_m
            for sample in trajectory.samples
        )
        successes.append(success)

    return CatchabilityResult(
        trial_successes=tuple(successes),
        flight_model_provenance=disc.aerodynamic_coefficients.provenance,
        throw_variation_provenance=variation_profile.provenance,
    )
