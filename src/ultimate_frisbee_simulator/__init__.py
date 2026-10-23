"""Local Ultimate Frisbee simulation domain models."""

from .aerodynamics import (
    AerodynamicCoefficientModel,
    AerodynamicWrench,
    CoefficientSurface,
    evaluate_aerodynamics,
    provisional_aerodynamic_coefficients,
    rotate_body_to_world,
)
from .catchability import (
    CatchabilityResult,
    ThrowEvaluationError,
    evaluate_flight_catchability,
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
from .scenario_editor import (
    PassPlanningMode,
    ScenarioEditor,
    create_scenario_editor,
    show_scenario_editor,
)
from .throw_evaluation import (
    AutomaticPassResult,
    AutomaticTimingSettings,
    NominalReleaseResult,
    ReleaseFailure,
    plan_target_pass,
    solve_nominal_release,
)
from .throwing import (
    PlannedPass,
    TargetPassRequest,
    ThrowProfile,
    ThrowType,
    default_throw_profiles,
)
from .throw_variation import (
    ThrowVariationProfile,
    default_throw_variation_profiles,
    sample_throw_execution,
)
from .visualization import animate_flight

__all__ = [
    "AerodynamicCoefficientModel",
    "AerodynamicWrench",
    "animate_flight",
    "AutomaticPassResult",
    "AutomaticTimingSettings",
    "CatchabilityResult",
    "CoefficientSurface",
    "DiscProfile",
    "FlightEnvironment",
    "FieldGeometry",
    "FlightInitialState",
    "FlightTerminationReason",
    "FlightTrajectory",
    "FieldRegion",
    "NominalReleaseResult",
    "PassPlanningMode",
    "PlannedPass",
    "ReleaseFailure",
    "ScenarioEditor",
    "SolverSettings",
    "TargetPassRequest",
    "ThrowProfile",
    "ThrowType",
    "ThrowEvaluationError",
    "ThrowVariationProfile",
    "TrajectorySample",
    "discraft_ultra_star",
    "default_throw_profiles",
    "default_throw_variation_profiles",
    "create_scenario_editor",
    "evaluate_flight_catchability",
    "evaluate_aerodynamics",
    "provisional_aerodynamic_coefficients",
    "plan_target_pass",
    "rotate_body_to_world",
    "simulate_flight",
    "sample_throw_execution",
    "solve_nominal_release",
    "show_scenario_editor",
]
