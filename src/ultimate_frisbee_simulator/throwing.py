"""Validated tactical pass plans and nominal throw constraints."""

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from .aerodynamics import QuaternionWXYZ
from .flight import Vector3


def _finite(value: Real, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _point2(values: tuple[Real, ...], name: str) -> tuple[float, float]:
    if len(values) != 2:
        raise ValueError(f"{name} must contain exactly two coordinates")
    return _finite(values[0], name), _finite(values[1], name)


def _vector3(values: tuple[Real, ...], name: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{name} must contain exactly three components")
    return (
        _finite(values[0], name),
        _finite(values[1], name),
        _finite(values[2], name),
    )


class ThrowType(Enum):
    BACKHAND = "backhand"
    FOREHAND = "forehand"


def _validate_pass_geometry(planned: "PlannedPass | TargetPassRequest") -> None:
    for name in ("throw_origin_xy_m", "receiver_location_xy_m"):
        object.__setattr__(planned, name, _point2(getattr(planned, name), name))
    for name in (
        "release_height_m",
        "catch_height_m",
        "catch_radius_m",
        "catch_time_tolerance_s",
        "target_tolerance_m",
    ):
        value = _finite(getattr(planned, name), name)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        object.__setattr__(planned, name, value)


@dataclass(frozen=True)
class PlannedPass:
    """Nominal pass intent, independent from any particular execution."""

    throw_origin_xy_m: tuple[float, float]
    receiver_location_xy_m: tuple[float, float]
    nominal_catch_time_s: float
    release_height_m: float = 1.5
    catch_height_m: float = 1.0
    catch_radius_m: float = 1.25
    catch_time_tolerance_s: float = 0.15
    target_tolerance_m: float = 0.5

    def __post_init__(self) -> None:
        _validate_pass_geometry(self)
        catch_time = _finite(self.nominal_catch_time_s, "nominal_catch_time_s")
        if catch_time <= 0:
            raise ValueError("nominal_catch_time_s must be positive")
        object.__setattr__(self, "nominal_catch_time_s", catch_time)

    @property
    def throw_origin_m(self) -> Vector3:
        return (*self.throw_origin_xy_m, self.release_height_m)

    @property
    def catch_target_m(self) -> Vector3:
        return (*self.receiver_location_xy_m, self.catch_height_m)


@dataclass(frozen=True)
class ThrowProfile:
    """Constraints and nominal spin for one throw type."""

    throw_type: ThrowType
    minimum_speed_m_s: float = 5.0
    maximum_speed_m_s: float = 25.0
    minimum_elevation_rad: float = math.radians(-10.0)
    maximum_elevation_rad: float = math.radians(55.0)
    maximum_aim_offset_rad: float = math.radians(45.0)
    nominal_spin_rad_s: float = 50.0
    orientation_wxyz: QuaternionWXYZ = (1.0, 0.0, 0.0, 0.0)
    provenance: str = "Provisional generic release constraints; calibrate with measured throws."

    def __post_init__(self) -> None:
        if not isinstance(self.throw_type, ThrowType):
            raise ValueError("throw_type must be a ThrowType")
        for name in (
            "minimum_speed_m_s",
            "maximum_speed_m_s",
            "minimum_elevation_rad",
            "maximum_elevation_rad",
            "maximum_aim_offset_rad",
            "nominal_spin_rad_s",
        ):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if self.minimum_speed_m_s <= 0:
            raise ValueError("minimum_speed_m_s must be positive")
        if self.maximum_speed_m_s <= self.minimum_speed_m_s:
            raise ValueError("maximum_speed_m_s must exceed minimum_speed_m_s")
        if self.maximum_elevation_rad <= self.minimum_elevation_rad:
            raise ValueError("maximum_elevation_rad must exceed minimum_elevation_rad")
        if not 0 < self.maximum_aim_offset_rad <= math.pi:
            raise ValueError("maximum_aim_offset_rad must be in (0, pi]")
        if self.nominal_spin_rad_s == 0:
            raise ValueError("nominal_spin_rad_s must be non-zero")
        if len(self.orientation_wxyz) != 4:
            raise ValueError("orientation_wxyz must contain four components")
        quaternion = tuple(
            _finite(component, "orientation_wxyz")
            for component in self.orientation_wxyz
        )
        norm = math.sqrt(sum(component**2 for component in quaternion))
        if not math.isclose(norm, 1.0, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("orientation_wxyz must be a unit quaternion")
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError("provenance must not be empty")
        object.__setattr__(self, "orientation_wxyz", quaternion)


@dataclass(frozen=True)
class TargetPassRequest:
    """Target-only intent; a verified release determines its catch time."""

    throw_origin_xy_m: tuple[float, float]
    receiver_location_xy_m: tuple[float, float]
    throw_profile: ThrowProfile
    release_height_m: float = 1.5
    catch_height_m: float = 1.0
    catch_radius_m: float = 1.25
    catch_time_tolerance_s: float = 0.15
    target_tolerance_m: float = 0.5

    def __post_init__(self) -> None:
        _validate_pass_geometry(self)
        if not isinstance(self.throw_profile, ThrowProfile):
            raise ValueError("throw_profile must be a ThrowProfile")

    def at_catch_time(self, catch_time_s: float) -> PlannedPass:
        """Create a fixed-time candidate without mutating the request."""
        return PlannedPass(
            throw_origin_xy_m=self.throw_origin_xy_m,
            receiver_location_xy_m=self.receiver_location_xy_m,
            nominal_catch_time_s=catch_time_s,
            release_height_m=self.release_height_m,
            catch_height_m=self.catch_height_m,
            catch_radius_m=self.catch_radius_m,
            catch_time_tolerance_s=self.catch_time_tolerance_s,
            target_tolerance_m=self.target_tolerance_m,
        )


def default_throw_profiles() -> dict[ThrowType, ThrowProfile]:
    """Return provisional backhand and forehand release profiles."""
    return {
        ThrowType.BACKHAND: ThrowProfile(
            throw_type=ThrowType.BACKHAND,
            nominal_spin_rad_s=50.0,
        ),
        ThrowType.FOREHAND: ThrowProfile(
            throw_type=ThrowType.FOREHAND,
            nominal_spin_rad_s=-50.0,
        ),
    }
