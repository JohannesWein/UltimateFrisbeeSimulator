import math
import unittest

from ultimate_frisbee_simulator import (
    AerodynamicCoefficientModel,
    CoefficientSurface,
    FlightEnvironment,
    FlightInitialState,
    FlightTerminationReason,
    SolverSettings,
    discraft_ultra_star,
    simulate_flight,
)


def zero_aerodynamic_model():
    axis = (-math.pi, math.pi)
    zero_surface = CoefficientSurface(
        angle_of_attack_rad=axis,
        sideslip_rad=axis,
        values=((0.0, 0.0), (0.0, 0.0)),
    )
    return AerodynamicCoefficientModel(
        lift=zero_surface,
        drag=zero_surface,
        side_force=zero_surface,
        roll_moment=zero_surface,
        pitch_moment=zero_surface,
        yaw_moment=zero_surface,
        provenance="Zero coefficients for integration tests",
    )


def disc_with_zero_aerodynamics():
    disc = discraft_ultra_star()
    return type(disc)(
        name=disc.name,
        mass_kg=disc.mass_kg,
        diameter_m=disc.diameter_m,
        reference_area_m2=disc.reference_area_m2,
        principal_moments_kg_m2=disc.principal_moments_kg_m2,
        parameter_sources=disc.parameter_sources,
        aerodynamic_coefficients=zero_aerodynamic_model(),
    )


class FlightSimulationTests(unittest.TestCase):
    def test_ground_contact_terminates_flight(self):
        trajectory = simulate_flight(
            disc=disc_with_zero_aerodynamics(),
            initial_state=FlightInitialState(
                position_m=(0.0, 0.0, 0.5),
                velocity_m_s=(0.0, 0.0, 0.0),
            ),
            settings=SolverSettings(maximum_duration_s=2.0),
        )

        self.assertIs(
            trajectory.termination_reason,
            FlightTerminationReason.GROUND_CONTACT,
        )
        self.assertAlmostEqual(trajectory.samples[-1].position_m[2], 0.0, places=8)
        self.assertGreater(trajectory.samples[-1].elapsed_s, 0.0)

    def test_maximum_duration_terminates_flight(self):
        trajectory = simulate_flight(
            disc=disc_with_zero_aerodynamics(),
            initial_state=FlightInitialState(
                position_m=(0.0, 0.0, 100.0),
                velocity_m_s=(0.0, 0.0, 0.0),
            ),
            settings=SolverSettings(maximum_duration_s=0.1),
        )

        self.assertIs(
            trajectory.termination_reason,
            FlightTerminationReason.MAXIMUM_DURATION,
        )
        self.assertAlmostEqual(trajectory.samples[-1].elapsed_s, 0.1)

    def test_spin_changes_orientation_and_curves_flight(self):
        disc = discraft_ultra_star()
        launch = FlightInitialState(
            position_m=(0.0, 0.0, 2.0),
            velocity_m_s=(12.0, 0.0, 3.0),
            angular_velocity_body_rad_s=(0.0, 0.0, 50.0),
        )
        settings = SolverSettings(maximum_duration_s=2.0)

        trajectory = simulate_flight(disc, launch, settings=settings)

        self.assertGreater(len(trajectory.samples), 2)
        self.assertGreater(
            abs(trajectory.samples[-1].orientation_wxyz[1])
            + abs(trajectory.samples[-1].orientation_wxyz[2]),
            1e-4,
        )
        self.assertGreater(abs(trajectory.samples[-1].position_m[1]), 1e-4)
        self.assertIs(
            trajectory.termination_reason,
            FlightTerminationReason.GROUND_CONTACT,
        )

    def test_repeated_inputs_produce_identical_samples(self):
        args = (
            disc_with_zero_aerodynamics(),
            FlightInitialState(
                position_m=(0.0, 0.0, 1.0),
                velocity_m_s=(4.0, 0.0, 3.0),
                angular_velocity_body_rad_s=(0.0, 0.0, 10.0),
            ),
        )
        settings = SolverSettings(maximum_duration_s=0.5)

        first = simulate_flight(*args, settings=settings)
        second = simulate_flight(*args, settings=settings)

        self.assertEqual(first, second)

    def test_rejects_launch_below_ground(self):
        with self.assertRaisesRegex(ValueError, "below ground"):
            simulate_flight(
                disc_with_zero_aerodynamics(),
                FlightInitialState(
                    position_m=(0.0, 0.0, -0.1),
                    velocity_m_s=(1.0, 0.0, 0.0),
                ),
            )


if __name__ == "__main__":
    unittest.main()
