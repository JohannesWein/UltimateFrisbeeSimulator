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
    rotate_body_to_world,
)
from tests.test_aerodynamics import coefficient_model, disc_with_coefficients


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
    def test_phase_equivalent_trajectories(self):
        tilt = 0.2
        base_q = (math.cos(tilt/2), 0.0, math.sin(tilt/2), 0.0)
        base_omega = (1.0, 2.0, 50.0)
        times = (0.1, 0.3, 0.5)
        reference = None
        for phase in (0, 45, 90, 180, 270):
            angle = math.radians(phase)
            c, s = math.cos(angle/2), math.sin(angle/2)
            q = (base_q[0]*c, base_q[2]*s, base_q[2]*c, base_q[0]*s)
            omega = (
                math.cos(angle)*base_omega[0] + math.sin(angle)*base_omega[1],
                -math.sin(angle)*base_omega[0] + math.cos(angle)*base_omega[1],
                base_omega[2],
            )
            trajectory = simulate_flight(
                discraft_ultra_star(),
                FlightInitialState(
                    position_m=(0.0, 0.0, 20.0),
                    velocity_m_s=(18.0, 2.0, 1.0),
                    orientation_wxyz=q,
                    angular_velocity_body_rad_s=omega,
                ),
                environment=FlightEnvironment(wind_velocity_m_s=(2.0, -1.0, 0.5)),
                settings=SolverSettings(maximum_duration_s=0.5),
                sample_times_s=times,
            )
            samples = [next(s for s in trajectory.samples if s.elapsed_s == t) for t in times]
            if reference is None:
                reference = samples
            for sample, expected in zip(samples, reference):
                with self.subTest(phase=phase, time=sample.elapsed_s):
                    self.assertLessEqual(math.dist(sample.position_m, expected.position_m), 1e-4)
                    self.assertLessEqual(math.dist(sample.velocity_m_s, expected.velocity_m_s), 1e-4)
                    for field, tolerance in (
                        ("normal", 1e-5), ("omega", 1e-3),
                    ):
                        def world_vector(value):
                            vector = ((0.0, 0.0, 1.0) if field == "normal"
                                      else value.angular_velocity_body_rad_s)
                            return rotate_body_to_world(value.orientation_wxyz, vector)
                        self.assertLessEqual(
                            math.dist(world_vector(sample), world_vector(expected)), tolerance,
                        )

    def test_spin_direction_and_rate_change_gyroscopic_response(self):
        disc = disc_with_coefficients(coefficient_model(pitch=0.01))
        normals = {}
        for spin in (25.0, 50.0, -50.0):
            trajectory = simulate_flight(
                disc,
                FlightInitialState(
                    position_m=(0.0, 0.0, 20.0),
                    velocity_m_s=(15.0, 0.0, 0.0),
                    angular_velocity_body_rad_s=(0.0, 0.0, spin),
                ),
                settings=SolverSettings(maximum_duration_s=0.3),
            )
            normals[spin] = rotate_body_to_world(
                trajectory.samples[-1].orientation_wxyz, (0.0, 0.0, 1.0),
            )
            self.assertGreater(math.dist(normals[spin], (0.0, 0.0, 1.0)), 1e-4)
        self.assertLess(normals[50.0][1]*normals[-50.0][1], 0.0)
        self.assertGreater(math.dist(normals[25.0], normals[50.0]), 1e-4)

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

    def test_requested_sample_is_interpolated_at_exact_time(self):
        trajectory = simulate_flight(
            disc_with_zero_aerodynamics(),
            FlightInitialState(
                position_m=(0.0, 0.0, 10.0),
                velocity_m_s=(10.0, 0.0, 0.0),
            ),
            settings=SolverSettings(maximum_duration_s=1.0),
            sample_times_s=(0.333,),
        )

        sample = next(
            sample for sample in trajectory.samples if sample.elapsed_s == 0.333
        )
        self.assertAlmostEqual(sample.position_m[0], 3.33, places=7)

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
