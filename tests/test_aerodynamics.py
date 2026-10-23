import math
import unittest

from ultimate_frisbee_simulator import (
    AerodynamicCoefficientModel,
    CoefficientSurface,
    discraft_ultra_star,
    evaluate_aerodynamics,
    rotate_body_to_world,
)


def constant_surface(value):
    axis = (-1.0, 1.0)
    return CoefficientSurface(axis, axis, ((value, value), (value, value)))


def coefficient_model(*, lift=0.0, drag=0.0, side=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return AerodynamicCoefficientModel(
        lift=constant_surface(lift),
        drag=constant_surface(drag),
        side_force=constant_surface(side),
        roll_moment=constant_surface(roll),
        pitch_moment=constant_surface(pitch),
        yaw_moment=constant_surface(yaw),
        provenance="Test coefficients",
    )


def disc_with_coefficients(model):
    reference = discraft_ultra_star()
    return type(reference)(
        name=reference.name,
        mass_kg=reference.mass_kg,
        diameter_m=reference.diameter_m,
        reference_area_m2=reference.reference_area_m2,
        principal_moments_kg_m2=reference.principal_moments_kg_m2,
        parameter_sources=reference.parameter_sources,
        aerodynamic_coefficients=model,
    )


class AerodynamicTests(unittest.TestCase):
    def test_coefficient_surface_bilinearly_interpolates_and_clamps(self):
        surface = CoefficientSurface(
            angle_of_attack_rad=(0.0, 1.0),
            sideslip_rad=(0.0, 1.0),
            values=((0.0, 1.0), (1.0, 2.0)),
        )

        self.assertEqual(surface.evaluate(0.5, 0.5), 1.0)
        self.assertEqual(surface.evaluate(-1.0, 0.5), 0.5)
        self.assertEqual(surface.evaluate(2.0, 2.0), 2.0)

    def test_coefficient_surface_rejects_invalid_axes_and_values(self):
        invalid = (
            {"angle_of_attack_rad": (0.0, 0.0)},
            {"sideslip_rad": (0.0, math.inf)},
            {"values": ((0.0,), (1.0,))},
        )
        for override in invalid:
            kwargs = {
                "angle_of_attack_rad": (0.0, 1.0),
                "sideslip_rad": (0.0, 1.0),
                "values": ((0.0, 0.0), (0.0, 0.0)),
                **override,
            }
            with self.subTest(override=override), self.assertRaises(ValueError):
                CoefficientSurface(**kwargs)

    def test_lift_and_drag_follow_expected_body_axes(self):
        disc = disc_with_coefficients(coefficient_model(lift=1.0, drag=1.0))

        wrench = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=(10.0, 0.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            wind_velocity_m_s=(0.0, 0.0, 0.0),
            air_density_kg_m3=1.225,
        )

        self.assertLess(wrench.force_body_n[0], 0.0)
        self.assertGreater(wrench.force_body_n[2], 0.0)
        self.assertAlmostEqual(wrench.force_body_n[1], 0.0)

    def test_side_force_opposes_positive_sideslip(self):
        disc = disc_with_coefficients(coefficient_model(side=-1.0))

        wrench = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=(10.0, 2.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            wind_velocity_m_s=(0.0, 0.0, 0.0),
            air_density_kg_m3=1.225,
        )

        self.assertGreater(wrench.sideslip_rad, 0.0)
        self.assertLess(wrench.force_body_n[1], 0.0)

    def test_positive_moment_coefficients_follow_body_axis_order(self):
        disc = disc_with_coefficients(
            coefficient_model(roll=1.0, pitch=1.0, yaw=1.0)
        )

        wrench = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=(10.0, 0.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            wind_velocity_m_s=(0.0, 0.0, 0.0),
            air_density_kg_m3=1.225,
        )

        self.assertTrue(all(moment > 0 for moment in wrench.moment_body_nm))

    def test_wind_relative_velocity_removes_forces_when_disc_matches_wind(self):
        disc = discraft_ultra_star()

        still_air = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=(10.0, 0.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            wind_velocity_m_s=(0.0, 0.0, 0.0),
            air_density_kg_m3=1.225,
        )
        matching_wind = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=(10.0, 0.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            wind_velocity_m_s=(10.0, 0.0, 0.0),
            air_density_kg_m3=1.225,
        )

        self.assertGreater(math.sqrt(sum(value**2 for value in still_air.force_body_n)), 0)
        self.assertEqual(matching_wind.force_body_n, (0.0, 0.0, 0.0))
        self.assertEqual(matching_wind.moment_body_nm, (0.0, 0.0, 0.0))

    def test_quaternion_rotation_transforms_body_normal(self):
        quarter_turn_about_y = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)

        rotated = rotate_body_to_world(quarter_turn_about_y, (0.0, 0.0, 1.0))

        self.assertAlmostEqual(rotated[0], 1.0)
        self.assertAlmostEqual(rotated[1], 0.0)
        self.assertAlmostEqual(rotated[2], 0.0)


if __name__ == "__main__":
    unittest.main()
