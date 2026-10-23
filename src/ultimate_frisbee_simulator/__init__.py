"""Local Ultimate Frisbee simulation domain models."""

from .aerodynamics import (
    AerodynamicCoefficientModel,
    AerodynamicWrench,
    CoefficientSurface,
    evaluate_aerodynamics,
    provisional_aerodynamic_coefficients,
    rotate_body_to_world,
)
from .field import FieldGeometry, FieldRegion
from .flight import (
    DiscProfile,
    FlightEnvironment,
    FlightInitialState,
    SolverSettings,
    discraft_ultra_star,
)
from .simulation import (
    FlightTerminationReason,
    FlightTrajectory,
    TrajectorySample,
    simulate_flight,
)
from .visualization import animate_flight

__all__ = [
    "AerodynamicCoefficientModel",
    "AerodynamicWrench",
    "animate_flight",
    "CoefficientSurface",
    "DiscProfile",
    "FlightEnvironment",
    "FieldGeometry",
    "FlightInitialState",
    "FlightTerminationReason",
    "FlightTrajectory",
    "FieldRegion",
    "SolverSettings",
    "TrajectorySample",
    "discraft_ultra_star",
    "evaluate_aerodynamics",
    "provisional_aerodynamic_coefficients",
    "rotate_body_to_world",
    "simulate_flight",
]
