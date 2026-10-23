import math
import unittest

from ultimate_frisbee_simulator import (
    DiscProfile,
    FlightEnvironment,
    FlightInitialState,
    SolverSettings,
    discraft_ultra_star,
)


class FlightInputTests(unittest.TestCase):
    def test_ultra_star_profile_has_traceable_parameters(self):
        disc = discraft_ultra_star()

        self.assertEqual(disc.name, "Discraft Ultra-Star")
        self.assertEqual(disc.mass_kg, 0.175)
        self.assertEqual(disc.diameter_m, 0.273)
        sources = dict(disc.parameter_sources)
        self.assertIn("Discraft", sources["mass_kg"])
        self.assertIn("Assumption", sources["principal_moments_kg_m2"])
        self.assertGreater(disc.reference_area_m2, 0)
        self.assertTrue(all(moment > 0 for moment in disc.principal_moments_kg_m2))

    def test_disc_profile_requires_provenance_for_each_parameter(self):
        with self.assertRaises(ValueError):
            DiscProfile(
                name="test disc",
                mass_kg=0.175,
                diameter_m=0.273,
                reference_area_m2=0.0585,
                principal_moments_kg_m2=(0.001, 0.001, 0.002),
                parameter_sources=(("mass_kg", "test source"),),
            )

    def test_disc_profile_rejects_non_positive_or_non_finite_values(self):
        disc = discraft_ultra_star()
        for kwargs in (
            {"mass_kg": 0.0},
            {"diameter_m": -1.0},
            {"reference_area_m2": math.inf},
            {"principal_moments_kg_m2": (0.001, 0.0, 0.002)},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DiscProfile(
                    name=disc.name,
                    mass_kg=kwargs.get("mass_kg", disc.mass_kg),
                    diameter_m=kwargs.get("diameter_m", disc.diameter_m),
                    reference_area_m2=kwargs.get(
                        "reference_area_m2", disc.reference_area_m2
                    ),
                    principal_moments_kg_m2=kwargs.get(
                        "principal_moments_kg_m2", disc.principal_moments_kg_m2
                    ),
                    parameter_sources=disc.parameter_sources,
                )

    def test_initial_state_accepts_unit_quaternion_and_spin(self):
        state = FlightInitialState(
            position_m=(1, 2, 3),
            velocity_m_s=(10, 0, 2),
            orientation_wxyz=(1, 0, 0, 0),
            angular_velocity_body_rad_s=(0, 0, 20),
        )

        self.assertEqual(state.position_m, (1.0, 2.0, 3.0))
        self.assertEqual(state.angular_velocity_body_rad_s, (0.0, 0.0, 20.0))

    def test_initial_state_rejects_non_finite_or_malformed_vectors(self):
        for kwargs in (
            {"position_m": (0, math.nan, 0)},
            {"velocity_m_s": (1, 2)},
            {"orientation_wxyz": (1, 0, 0)},
            {"orientation_wxyz": (2, 0, 0, 0)},
            {"angular_velocity_body_rad_s": (0, 0, math.inf)},
        ):
            values = {
                "position_m": (0, 0, 1),
                "velocity_m_s": (10, 0, 0),
                **kwargs,
            }
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FlightInitialState(**values)

    def test_environment_rejects_invalid_inputs(self):
        for kwargs in (
            {"wind_velocity_m_s": (0, math.inf, 0)},
            {"air_density_kg_m3": 0},
            {"gravity_m_s2": math.nan},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FlightEnvironment(**kwargs)

    def test_solver_settings_reject_invalid_inputs(self):
        for kwargs in (
            {"relative_tolerance": 0},
            {"absolute_tolerance": -1},
            {"maximum_step_s": math.inf},
            {"maximum_duration_s": 0},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SolverSettings(**kwargs)


if __name__ == "__main__":
    unittest.main()
