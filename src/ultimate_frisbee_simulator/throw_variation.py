"""Configurable, bounded sampling of human throw-execution variation."""

from dataclasses import dataclass
import math
from numbers import Real

import numpy as np
from scipy.stats import truncnorm

from .flight import FlightInitialState
from .throw_evaluation import NominalReleaseResult
from .throwing import PlannedPass, ThrowProfile, ThrowType


def _nonnegative(value: Real, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite non-negative number")
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ValueError(f"{name} must be a finite non-negative number")
    return result


@dataclass(frozen=True)
class ThrowVariationProfile:
    """Zero-distance spreads and their linear increase per throw metre."""

    throw_type: ThrowType
    speed_stddev_m_s: float
    speed_stddev_increase_per_m_m_s: float
    direction_stddev_rad: float
    direction_stddev_increase_per_m_rad: float
    elevation_stddev_rad: float
    elevation_stddev_increase_per_m_rad: float
    spin_stddev_rad_s: float
    spin_stddev_increase_per_m_rad_s: float
    provenance: str = "Provisional generic variation; calibrate with measured throws."

    def __post_init__(self) -> None:
        if not isinstance(self.throw_type, ThrowType):
            raise ValueError("throw_type must be a ThrowType")
        for name in (
            "speed_stddev_m_s",
            "speed_stddev_increase_per_m_m_s",
            "direction_stddev_rad",
            "direction_stddev_increase_per_m_rad",
            "elevation_stddev_rad",
            "elevation_stddev_increase_per_m_rad",
            "spin_stddev_rad_s",
            "spin_stddev_increase_per_m_rad_s",
        ):
            object.__setattr__(self, name, _nonnegative(getattr(self, name), name))
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError("provenance must not be empty")

    def standard_deviations(self, distance_m: float) -> tuple[float, float, float, float]:
        """Return speed, direction, elevation, and spin spreads at a distance."""
        distance = _nonnegative(distance_m, "distance_m")
        return (
            self.speed_stddev_m_s
            + self.speed_stddev_increase_per_m_m_s * distance,
            self.direction_stddev_rad
            + self.direction_stddev_increase_per_m_rad * distance,
            self.elevation_stddev_rad
            + self.elevation_stddev_increase_per_m_rad * distance,
            self.spin_stddev_rad_s
            + self.spin_stddev_increase_per_m_rad_s * distance,
        )


def default_throw_variation_profiles() -> dict[ThrowType, ThrowVariationProfile]:
    """Return explicitly provisional backhand and forehand variation spreads."""
    return {
        ThrowType.BACKHAND: ThrowVariationProfile(
            throw_type=ThrowType.BACKHAND,
            speed_stddev_m_s=0.2,
            speed_stddev_increase_per_m_m_s=0.025,
            direction_stddev_rad=0.005,
            direction_stddev_increase_per_m_rad=0.0005,
            elevation_stddev_rad=0.005,
            elevation_stddev_increase_per_m_rad=0.00025,
            spin_stddev_rad_s=2.0,
            spin_stddev_increase_per_m_rad_s=0.1,
        ),
        ThrowType.FOREHAND: ThrowVariationProfile(
            throw_type=ThrowType.FOREHAND,
            speed_stddev_m_s=0.3,
            speed_stddev_increase_per_m_m_s=0.03,
            direction_stddev_rad=0.008,
            direction_stddev_increase_per_m_rad=0.0007,
            elevation_stddev_rad=0.007,
            elevation_stddev_increase_per_m_rad=0.00035,
            spin_stddev_rad_s=3.0,
            spin_stddev_increase_per_m_rad_s=0.12,
        ),
    }


def _truncated_normal(
    generator: np.random.Generator,
    standard_deviation: float,
    minimum: float,
    maximum: float,
) -> float:
    if standard_deviation == 0:
        return 0.0
    lower = max(minimum, -3.0 * standard_deviation)
    upper = min(maximum, 3.0 * standard_deviation)
    if lower > upper:
        raise ValueError("variation bounds do not contain an allowed deviation")
    if lower == upper:
        return lower
    return float(
        truncnorm.rvs(
            lower / standard_deviation,
            upper / standard_deviation,
            loc=0.0,
            scale=standard_deviation,
            random_state=generator,
        )
    )


def sample_throw_execution(
    planned_pass: PlannedPass,
    nominal_release: NominalReleaseResult,
    throw_profile: ThrowProfile,
    variation_profile: ThrowVariationProfile,
    generator: np.random.Generator,
) -> FlightInitialState:
    """Sample one bounded release state without changing its tactical plan."""
    if not isinstance(generator, np.random.Generator):
        raise TypeError("generator must be a numpy.random.Generator")
    if not nominal_release.feasible or nominal_release.launch_state is None:
        raise ValueError("nominal_release must contain a feasible launch state")
    if throw_profile.throw_type is not variation_profile.throw_type:
        raise ValueError("throw and variation profiles must have the same throw_type")

    launch = nominal_release.launch_state
    target_delta = (
        planned_pass.receiver_location_xy_m[0] - planned_pass.throw_origin_xy_m[0],
        planned_pass.receiver_location_xy_m[1] - planned_pass.throw_origin_xy_m[1],
    )
    target_heading = math.atan2(target_delta[1], target_delta[0])
    speed = math.sqrt(sum(component**2 for component in launch.velocity_m_s))
    horizontal_speed = math.hypot(*launch.velocity_m_s[:2])
    heading = math.atan2(launch.velocity_m_s[1], launch.velocity_m_s[0])
    elevation = math.atan2(launch.velocity_m_s[2], horizontal_speed)
    aim_offset = math.atan2(
        math.sin(heading - target_heading),
        math.cos(heading - target_heading),
    )
    spin = launch.angular_velocity_body_rad_s[2]
    distance = math.hypot(*target_delta)
    speed_spread, direction_spread, elevation_spread, spin_spread = (
        variation_profile.standard_deviations(distance)
    )

    speed_delta = _truncated_normal(
        generator,
        speed_spread,
        throw_profile.minimum_speed_m_s - speed,
        throw_profile.maximum_speed_m_s - speed,
    )
    direction_delta = _truncated_normal(
        generator,
        direction_spread,
        -throw_profile.maximum_aim_offset_rad - aim_offset,
        throw_profile.maximum_aim_offset_rad - aim_offset,
    )
    elevation_delta = _truncated_normal(
        generator,
        elevation_spread,
        throw_profile.minimum_elevation_rad - elevation,
        throw_profile.maximum_elevation_rad - elevation,
    )
    spin_minimum = 1e-6 if spin > 0 else -math.inf
    spin_maximum = -1e-6 if spin < 0 else math.inf
    if spin > 0:
        spin_minimum -= spin
        spin_maximum = math.inf
    else:
        spin_minimum = -math.inf
        spin_maximum += -spin
    spin_delta = _truncated_normal(
        generator,
        spin_spread,
        spin_minimum,
        spin_maximum,
    )

    sampled_speed = speed + speed_delta
    sampled_heading = heading + direction_delta
    sampled_elevation = elevation + elevation_delta
    horizontal_sampled_speed = sampled_speed * math.cos(sampled_elevation)
    return FlightInitialState(
        position_m=launch.position_m,
        velocity_m_s=(
            horizontal_sampled_speed * math.cos(sampled_heading),
            horizontal_sampled_speed * math.sin(sampled_heading),
            sampled_speed * math.sin(sampled_elevation),
        ),
        orientation_wxyz=launch.orientation_wxyz,
        angular_velocity_body_rad_s=(
            launch.angular_velocity_body_rad_s[0],
            launch.angular_velocity_body_rad_s[1],
            spin + spin_delta,
        ),
    )
