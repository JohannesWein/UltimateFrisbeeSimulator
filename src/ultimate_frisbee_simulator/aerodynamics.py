"""Configurable aerodynamic coefficient surfaces and disc wrench evaluation."""

from dataclasses import dataclass
import math
from numbers import Real
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .flight import DiscProfile

Vector3 = tuple[float, float, float]
QuaternionWXYZ = tuple[float, float, float, float]


def _finite(value: Real, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must contain only finite numbers")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must contain only finite numbers")
    return result


def _axis(values: tuple[Real, ...], name: str) -> tuple[float, ...]:
    result = tuple(_finite(value, name) for value in values)
    if len(result) < 2 or any(right <= left for left, right in zip(result, result[1:])):
        raise ValueError(f"{name} must contain at least two strictly increasing values")
    return result


def _bracket(axis: tuple[float, ...], value: float) -> tuple[int, int, float]:
    if value <= axis[0]:
        return 0, 0, 0.0
    if value >= axis[-1]:
        last = len(axis) - 1
        return last, last, 0.0
    for lower in range(len(axis) - 1):
        if value <= axis[lower + 1]:
            fraction = (value - axis[lower]) / (axis[lower + 1] - axis[lower])
            return lower, lower + 1, fraction
    raise RuntimeError("coefficient axis bracket was not found")


@dataclass(frozen=True)
class CoefficientSurface:
    """Bilinearly interpolated coefficient over attack and sideslip angles.

    Rows in ``values`` correspond to angle of attack; columns correspond to
    sideslip. Values beyond either axis are clamped to its endpoint.
    """

    angle_of_attack_rad: tuple[float, ...]
    sideslip_rad: tuple[float, ...]
    values: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        attack = _axis(self.angle_of_attack_rad, "angle_of_attack_rad")
        sideslip = _axis(self.sideslip_rad, "sideslip_rad")
        values = tuple(
            tuple(_finite(value, "values") for value in row)
            for row in self.values
        )
        if len(values) != len(attack) or any(len(row) != len(sideslip) for row in values):
            raise ValueError("values must have one row per attack angle and one column per sideslip")
        object.__setattr__(self, "angle_of_attack_rad", attack)
        object.__setattr__(self, "sideslip_rad", sideslip)
        object.__setattr__(self, "values", values)

    def evaluate(self, angle_of_attack_rad: float, sideslip_rad: float) -> float:
        attack_value = _finite(angle_of_attack_rad, "angle_of_attack_rad")
        sideslip_value = _finite(sideslip_rad, "sideslip_rad")
        a0, a1, attack_fraction = _bracket(self.angle_of_attack_rad, attack_value)
        b0, b1, sideslip_fraction = _bracket(self.sideslip_rad, sideslip_value)

        lower = self.values[a0][b0] + sideslip_fraction * (
            self.values[a0][b1] - self.values[a0][b0]
        )
        upper = self.values[a1][b0] + sideslip_fraction * (
            self.values[a1][b1] - self.values[a1][b0]
        )
        return lower + attack_fraction * (upper - lower)


@dataclass(frozen=True)
class AerodynamicCoefficientModel:
    """Six coefficient surfaces and their provenance."""

    lift: CoefficientSurface
    drag: CoefficientSurface
    side_force: CoefficientSurface
    roll_moment: CoefficientSurface
    pitch_moment: CoefficientSurface
    yaw_moment: CoefficientSurface
    provenance: str

    def __post_init__(self) -> None:
        for name in (
            "lift",
            "drag",
            "side_force",
            "roll_moment",
            "pitch_moment",
            "yaw_moment",
        ):
            if not isinstance(getattr(self, name), CoefficientSurface):
                raise ValueError(f"{name} must be a CoefficientSurface")
        if not isinstance(self.provenance, str) or not self.provenance.strip():
            raise ValueError("coefficient provenance must not be empty")


@dataclass(frozen=True)
class AerodynamicWrench:
    """Aerodynamic force and moment in the disc's body frame."""

    force_body_n: Vector3
    moment_body_nm: Vector3
    relative_air_velocity_body_m_s: Vector3
    angle_of_attack_rad: float
    sideslip_rad: float


def provisional_aerodynamic_coefficients() -> AerodynamicCoefficientModel:
    """Return illustrative, uncalibrated curves for exercising the simulator."""
    angles = tuple(math.radians(value) for value in (-30, -15, 0, 15, 30))
    zero = (0.0,) * len(angles)

    def surface(attack_values: tuple[float, ...], slip_values: tuple[float, ...]):
        return CoefficientSurface(
            angles,
            angles,
            tuple(
                tuple(attack + slip for slip in slip_values)
                for attack in attack_values
            ),
        )

    return AerodynamicCoefficientModel(
        lift=surface((-0.7, -0.35, 0.1, 0.85, 0.9), (-0.04, -0.01, 0.0, -0.01, -0.04)),
        drag=surface((0.55, 0.22, 0.12, 0.22, 0.55), (0.18, 0.05, 0.0, 0.05, 0.18)),
        side_force=surface(zero, (0.5, 0.25, 0.0, -0.25, -0.5)),
        roll_moment=surface(zero, (-0.02, -0.01, 0.0, 0.01, 0.02)),
        pitch_moment=surface((0.08, 0.04, 0.0, -0.04, -0.08), zero),
        yaw_moment=surface(zero, (0.04, 0.02, 0.0, -0.02, -0.04)),
        provenance=(
            "Illustrative provisional assumptions only; not digitized from "
            "Koyanagi et al. or validated for an UltraStar or Eurodisc."
        ),
    )


def rotate_body_to_world(
    orientation_wxyz: QuaternionWXYZ, vector_body: Vector3
) -> Vector3:
    """Rotate a body-frame vector into world coordinates using a quaternion."""
    if len(orientation_wxyz) != 4 or len(vector_body) != 3:
        raise ValueError("orientation and vector must have four and three components")
    w, x, y, z = (_finite(value, "orientation_wxyz") for value in orientation_wxyz)
    vx, vy, vz = (_finite(value, "vector_body") for value in vector_body)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        raise ValueError("orientation_wxyz must be non-zero")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _rotate_world_to_body(
    orientation_wxyz: QuaternionWXYZ, vector_world: Vector3
) -> Vector3:
    w, x, y, z = orientation_wxyz
    return rotate_body_to_world((w, -x, -y, -z), vector_world)


def evaluate_aerodynamics(
    disc: "DiscProfile",
    velocity_world_m_s: Vector3,
    orientation_wxyz: QuaternionWXYZ,
    wind_velocity_m_s: Vector3,
    air_density_kg_m3: float,
) -> AerodynamicWrench:
    """Evaluate forces and moments from wind-relative flow in the disc frame.

    Body x is the disc's forward direction, body y its span direction, and
    body z its normal. Lift is perpendicular to the flow and span; moments are
    returned around the body x, y, and z axes respectively.
    """
    if len(velocity_world_m_s) != 3 or len(wind_velocity_m_s) != 3:
        raise ValueError("velocity and wind vectors must have three components")
    velocity = tuple(_finite(value, "velocity_world_m_s") for value in velocity_world_m_s)
    wind = tuple(_finite(value, "wind_velocity_m_s") for value in wind_velocity_m_s)
    density = _finite(air_density_kg_m3, "air_density_kg_m3")
    if density <= 0:
        raise ValueError("air_density_kg_m3 must be positive")

    relative_world = tuple(velocity[index] - wind[index] for index in range(3))
    relative_body = _rotate_world_to_body(orientation_wxyz, relative_world)
    forward, spanwise, normal = relative_body
    speed = math.sqrt(forward**2 + spanwise**2 + normal**2)
    if speed == 0:
        return AerodynamicWrench((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), relative_body, 0.0, 0.0)

    angle_of_attack = math.atan2(normal, math.hypot(forward, spanwise))
    sideslip = math.atan2(spanwise, math.hypot(forward, normal))
    dynamic_pressure = 0.5 * density * speed**2
    area = disc.reference_area_m2
    coefficients = disc.aerodynamic_coefficients

    drag_scale = dynamic_pressure * area * coefficients.drag.evaluate(
        angle_of_attack, sideslip
    )
    drag = tuple(-drag_scale * component / speed for component in relative_body)

    lift_norm = math.hypot(forward, normal)
    if lift_norm == 0:
        lift = (0.0, 0.0, 0.0)
    else:
        lift_scale = dynamic_pressure * area * coefficients.lift.evaluate(
            angle_of_attack, sideslip
        )
        lift = (
            -normal * lift_scale / lift_norm,
            0.0,
            forward * lift_scale / lift_norm,
        )

    side_scale = dynamic_pressure * area * coefficients.side_force.evaluate(
        angle_of_attack, sideslip
    )
    force = (
        drag[0] + lift[0],
        drag[1] + side_scale,
        drag[2] + lift[2],
    )
    moment_scale = dynamic_pressure * area * disc.diameter_m
    moments = tuple(
        moment_scale * surface.evaluate(angle_of_attack, sideslip)
        for surface in (
            coefficients.roll_moment,
            coefficients.pitch_moment,
            coefficients.yaw_moment,
        )
    )
    return AerodynamicWrench(
        force_body_n=force,
        moment_body_nm=moments,
        relative_air_velocity_body_m_s=relative_body,
        angle_of_attack_rad=angle_of_attack,
        sideslip_rad=sideslip,
    )
