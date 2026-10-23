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
    def test_world_loads_are_spin_phase_invariant(self):
        for disc in (
            discraft_ultra_star(),
            disc_with_coefficients(coefficient_model(
                lift=0.5, drag=0.2, side=0.1, roll=0.02, pitch=-0.03, yaw=0.01,
            )),
        ):
            for tilt in (0.0, 0.3):
                for wind in ((0.0, 0.0, 0.0), (2.0, -3.0, 1.0)):
                    reference = None
                    for phase in (0, 45, 90, 180, 270):
                        half = math.radians(phase) / 2
                        q = (
                            math.cos(tilt / 2) * math.cos(half),
                            math.sin(tilt / 2) * math.sin(half),
                            math.sin(tilt / 2) * math.cos(half),
                            math.cos(tilt / 2) * math.sin(half),
                        )
                        wrench = evaluate_aerodynamics(
                            disc, (18.0, 0.0, 0.0), q, wind, 1.225,
                        )
                        loads = (
                            rotate_body_to_world(q, wrench.force_body_n),
                            rotate_body_to_world(q, wrench.moment_body_nm),
                        )
                        if reference is None:
                            reference = loads
                        with self.subTest(tilt=tilt, wind=wind, phase=phase):
                            for actual, expected in zip(loads, reference):
                                self.assertLessEqual(math.dist(actual, expected), 1e-9)

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

    def test_side_force_uses_flow_transverse_axis_at_zero_sideslip(self):
        disc = disc_with_coefficients(coefficient_model(side=-1.0))

        wrench = evaluate_aerodynamics(
            disc=disc,
            velocity_world_m_s=(10.0, 2.0, 0.0),
            orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            wind_velocity_m_s=(0.0, 0.0, 0.0),
            air_density_kg_m3=1.225,
        )

        self.assertEqual(wrench.sideslip_rad, 0.0)
        self.assertLess(wrench.force_body_n[1], 0.0)
        self.assertAlmostEqual(
            sum(a * b for a, b in zip(wrench.force_body_n, (10.0, 2.0, 0.0))), 0.0,
        )

    def test_attack_sign_and_lift_drag_geometry(self):
        for tilt in (-0.2, 0.2):
            q = (math.cos(tilt / 2), 0.0, math.sin(tilt / 2), 0.0)
            wrench = evaluate_aerodynamics(
                discraft_ultra_star(), (10.0, 0.0, 0.0), q, (0.0, 0.0, 0.0), 1.225,
            )
            self.assertAlmostEqual(wrench.angle_of_attack_rad, -tilt)
        velocity = (10.0, 3.0, 2.0)
        for channel in ("lift", "drag"):
            wrench = evaluate_aerodynamics(
                disc_with_coefficients(coefficient_model(**{channel: 1.0})),
                velocity, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.225,
            )
            force = wrench.force_body_n
            if channel == "lift":
                self.assertAlmostEqual(sum(a*b for a,b in zip(force, velocity)), 0.0)
                self.assertGreater(force[2], 0.0)
            else:
                ratios = [a/b for a,b in zip(force, velocity)]
                self.assertLess(ratios[0], 0.0)
                self.assertAlmostEqual(ratios[0], ratios[1])
                self.assertAlmostEqual(ratios[0], ratios[2])

    def test_normal_flow_is_finite_and_phase_independent(self):
        disc = disc_with_coefficients(coefficient_model(
            lift=1.0, drag=1.0, side=1.0, roll=1.0, pitch=1.0, yaw=1.0,
        ))
        for sign in (-1, 1):
            for ratio in (0.0, 0.5e-12, 2e-12):
                reference = None
                for phase in (0, 45, 90, 180, 270):
                    half = math.radians(phase)/2
                    q = (math.cos(half), 0.0, 0.0, math.sin(half))
                    wrench = evaluate_aerodynamics(
                        disc, (10*ratio, 0.0, sign*10.0), q,
                        (0.0, 0.0, 0.0), 1.225,
                    )
                    values = (*wrench.force_body_n, *wrench.moment_body_nm,
                              wrench.angle_of_attack_rad, wrench.sideslip_rad)
                    self.assertTrue(all(math.isfinite(value) for value in values))
                    loads = (
                        rotate_body_to_world(q, wrench.force_body_n),
                        rotate_body_to_world(q, wrench.moment_body_nm),
                    )
                    if reference is None:
                        reference = loads
                    for actual, expected in zip(loads, reference):
                        self.assertLessEqual(math.dist(actual, expected), 1e-9)
                    if ratio < 1e-12:
                        self.assertEqual(wrench.force_body_n[:2], (0.0, 0.0))
                        self.assertEqual(wrench.moment_body_nm[:2], (0.0, 0.0))

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
