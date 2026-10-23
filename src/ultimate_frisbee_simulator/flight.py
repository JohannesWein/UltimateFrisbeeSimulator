"""Validated physical inputs for disc-flight simulation."""

from dataclasses import dataclass, field
import math
from numbers import Real

from .aerodynamics import (
    AerodynamicCoefficientModel,
    provisional_aerodynamic_coefficients,
)

Vector3 = tuple[float, float, float]
QuaternionWXYZ = tuple[float, float, float, float]


def _finite_number(value: Real, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _vector3(value: tuple[Real, ...], name: str) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three components")
    return (
        _finite_number(value[0], name),
        _finite_number(value[1], name),
        _finite_number(value[2], name),
    )


@dataclass(frozen=True)
class DiscProfile:
    """Physical disc parameters and provenance for every configured value."""

    name: str
    mass_kg: float
    diameter_m: float
    reference_area_m2: float
    principal_moments_kg_m2: Vector3
    parameter_sources: tuple[tuple[str, str], ...]
    aerodynamic_coefficients: AerodynamicCoefficientModel = field(
        default_factory=provisional_aerodynamic_coefficients
    )

    def __post_init__(self) -> None:
        positive_values = {
            "mass_kg": self.mass_kg,
            "diameter_m": self.diameter_m,
            "reference_area_m2": self.reference_area_m2,
        }
        for name, value in positive_values.items():
            if _finite_number(value, name) <= 0:
                raise ValueError(f"{name} must be positive")

        moments = _vector3(self.principal_moments_kg_m2, "principal_moments_kg_m2")
        if any(moment <= 0 for moment in moments):
            raise ValueError("principal moments of inertia must be positive")
        object.__setattr__(self, "principal_moments_kg_m2", moments)

        if not self.name.strip():
            raise ValueError("disc name must not be empty")
        sources = dict(self.parameter_sources)
        required = {
            "mass_kg",
            "diameter_m",
            "reference_area_m2",
            "principal_moments_kg_m2",
        }
        if required - sources.keys():
            raise ValueError("every disc parameter must have a documented source or assumption")
        if any(not isinstance(source, str) or not source.strip() for source in sources.values()):
            raise ValueError("disc parameter sources must not be empty")
        object.__setattr__(self, "parameter_sources", tuple(sorted(sources.items())))
        if not isinstance(self.aerodynamic_coefficients, AerodynamicCoefficientModel):
            raise ValueError("aerodynamic_coefficients must be an AerodynamicCoefficientModel")


def discraft_ultra_star() -> DiscProfile:
    """Return a traceable UltraStar profile with an explicitly assumed inertia."""
    mass_kg = 0.175
    diameter_m = 0.273
    radius_m = diameter_m / 2
    reference_area_m2 = math.pi * radius_m**2
    return DiscProfile(
        name="Discraft Ultra-Star",
        mass_kg=mass_kg,
        diameter_m=diameter_m,
        reference_area_m2=reference_area_m2,
        principal_moments_kg_m2=(
            0.25 * mass_kg * radius_m**2,
            0.25 * mass_kg * radius_m**2,
            0.5 * mass_kg * radius_m**2,
        ),
        parameter_sources=(
            (
                "mass_kg",
                "Discraft Ultra-Star product specification (175 g): "
                "https://discraft.com/ultimate.html",
            ),
            (
                "diameter_m",
                "Discraft Ultra-Star product specification (27.3 cm): "
                "https://discraft.com/ultimate.html",
            ),
            ("reference_area_m2", "Derived from the stated 27.3 cm diameter as a circular planform."),
            (
                "principal_moments_kg_m2",
                "Assumption: uniform thin circular disk; measure or calibrate for the rimmed Ultra-Star.",
            ),
        ),
        aerodynamic_coefficients=provisional_aerodynamic_coefficients(),
    )


@dataclass(frozen=True)
class FlightInitialState:
    """Launch state in world coordinates and SI units.

    Attitude is a unit Hamilton quaternion in [w, x, y, z] order, rotating
    body-frame vectors into the world frame. Angular velocity is body-relative.
    """

    position_m: Vector3
    velocity_m_s: Vector3
    orientation_wxyz: QuaternionWXYZ = (1.0, 0.0, 0.0, 0.0)
    angular_velocity_body_rad_s: Vector3 = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        object.__setattr__(self, "position_m", _vector3(self.position_m, "position_m"))
        object.__setattr__(self, "velocity_m_s", _vector3(self.velocity_m_s, "velocity_m_s"))
        object.__setattr__(
            self,
            "angular_velocity_body_rad_s",
            _vector3(self.angular_velocity_body_rad_s, "angular_velocity_body_rad_s"),
        )
        if len(self.orientation_wxyz) != 4:
            raise ValueError("orientation_wxyz must contain exactly four components")
        orientation = tuple(
            _finite_number(component, "orientation_wxyz")
            for component in self.orientation_wxyz
        )
        norm = math.sqrt(sum(component**2 for component in orientation))
        if not math.isclose(norm, 1.0, rel_tol=1e-9, abs_tol=1e-9):
            raise ValueError("orientation_wxyz must be a unit quaternion")
        object.__setattr__(self, "orientation_wxyz", orientation)


@dataclass(frozen=True)
class FlightEnvironment:
    """Constant wind and atmospheric properties for one simulated flight."""

    wind_velocity_m_s: Vector3 = (0.0, 0.0, 0.0)
    air_density_kg_m3: float = 1.225
    gravity_m_s2: float = 9.80665

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "wind_velocity_m_s",
            _vector3(self.wind_velocity_m_s, "wind_velocity_m_s"),
        )
        for name in ("air_density_kg_m3", "gravity_m_s2"):
            if _finite_number(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class SolverSettings:
    """Numerical tolerances and duration limits for flight integration."""

    relative_tolerance: float = 1e-8
    absolute_tolerance: float = 1e-10
    maximum_step_s: float = 0.01
    maximum_duration_s: float = 10.0

    def __post_init__(self) -> None:
        for name in (
            "relative_tolerance",
            "absolute_tolerance",
            "maximum_step_s",
            "maximum_duration_s",
        ):
            if _finite_number(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
