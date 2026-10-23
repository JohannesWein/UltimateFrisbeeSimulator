import math
import unittest
from dataclasses import FrozenInstanceError

from ultimate_frisbee_simulator import (
    AutomaticTimingSettings,
    PlannedPass,
    TargetPassRequest,
    ThrowProfile,
    ThrowType,
)


class TargetPassRequestTests(unittest.TestCase):
    def test_resolving_time_preserves_geometry_and_catch_window(self):
        request = TargetPassRequest(
            (10, 5), (20, 7), ThrowProfile(ThrowType.BACKHAND),
            release_height_m=1.7, catch_height_m=1.2,
            catch_radius_m=0.7, catch_time_tolerance_s=0.1, target_tolerance_m=0.2,
        )
        resolved = request.at_catch_time(0.8)
        self.assertEqual(resolved.throw_origin_m, (10.0, 5.0, 1.7))
        self.assertEqual(resolved.catch_target_m, (20.0, 7.0, 1.2))
        self.assertEqual(resolved.nominal_catch_time_s, 0.8)
        self.assertEqual(resolved.catch_radius_m, 0.7)
        self.assertEqual(resolved.catch_time_tolerance_s, 0.1)
        self.assertEqual(resolved.target_tolerance_m, 0.2)
        with self.assertRaises(FrozenInstanceError):
            resolved.nominal_catch_time_s = 1.0
        with self.assertRaises(FrozenInstanceError):
            request.receiver_location_xy_m = (1, 2)

    def test_invalid_requests_are_reported(self):
        for override in (
            {"throw_origin_xy_m": (math.nan, 0)},
            {"receiver_location_xy_m": (0, math.inf)},
            {"catch_height_m": 0},
            {"catch_radius_m": -1},
            {"throw_profile": "backhand"},
        ):
            with self.subTest(override=override), self.assertRaises(ValueError):
                TargetPassRequest(**{
                    "throw_origin_xy_m": (0, 0),
                    "receiver_location_xy_m": (10, 0),
                    "throw_profile": ThrowProfile(ThrowType.BACKHAND),
                    **override,
                })

    def test_invalid_timing_search_settings_are_reported(self):
        for override in (
            {"minimum_catch_time_s": 0},
            {"minimum_catch_time_s": math.nan},
            {"maximum_catch_time_s": math.inf},
            {"maximum_catch_time_s": -1},
            {"minimum_catch_time_s": 3, "maximum_catch_time_s": 2},
            {"minimum_catch_time_s": 1, "maximum_catch_time_s": 1},
            {"candidate_count": 1},
            {"candidate_count": 2.5},
            {"candidate_count": True},
            {"maximum_evaluations_per_candidate": 0},
            {"maximum_evaluations_per_candidate": False},
            {"maximum_evaluations_per_candidate": math.inf},
        ):
            with self.subTest(override=override), self.assertRaises(ValueError):
                AutomaticTimingSettings(**override)


class PlannedPassTests(unittest.TestCase):
    def test_plan_keeps_target_and_catch_time_separate(self):
        planned = PlannedPass(
            throw_origin_xy_m=(20, 18.5),
            receiver_location_xy_m=(45, 12),
            nominal_catch_time_s=1.7,
        )

        self.assertEqual(planned.throw_origin_m, (20.0, 18.5, 1.5))
        self.assertEqual(planned.catch_target_m, (45.0, 12.0, 1.0))
        self.assertEqual(planned.receiver_location_xy_m, (45.0, 12.0))
        self.assertEqual(planned.nominal_catch_time_s, 1.7)

    def test_rejects_nonfinite_coordinates_and_nonpositive_timing(self):
        invalid_plans = (
            {"throw_origin_xy_m": (math.nan, 0.0)},
            {"receiver_location_xy_m": (0.0, math.inf)},
            {"nominal_catch_time_s": 0},
            {"nominal_catch_time_s": -1},
        )
        for override in invalid_plans:
            values = {
                "throw_origin_xy_m": (0.0, 0.0),
                "receiver_location_xy_m": (10.0, 5.0),
                "nominal_catch_time_s": 1.0,
                **override,
            }
            with self.subTest(override=override), self.assertRaises(ValueError):
                PlannedPass(**values)

    def test_rejects_nonpositive_catch_window_and_heights(self):
        for override in (
            {"catch_radius_m": 0.0},
            {"catch_time_tolerance_s": -0.1},
            {"target_tolerance_m": math.nan},
            {"release_height_m": 0.0},
        ):
            values = {
                "throw_origin_xy_m": (0.0, 0.0),
                "receiver_location_xy_m": (10.0, 5.0),
                "nominal_catch_time_s": 1.0,
                **override,
            }
            with self.subTest(override=override), self.assertRaises(ValueError):
                PlannedPass(**values)


class ThrowProfileTests(unittest.TestCase):
    def test_backhand_and_forehand_profiles_have_opposite_spin(self):
        backhand = ThrowProfile(throw_type=ThrowType.BACKHAND)
        forehand = ThrowProfile(
            throw_type=ThrowType.FOREHAND,
            nominal_spin_rad_s=-50,
        )

        self.assertGreater(backhand.nominal_spin_rad_s, 0)
        self.assertLess(forehand.nominal_spin_rad_s, 0)

    def test_rejects_invalid_throw_constraints(self):
        invalid_profiles = (
            {"minimum_speed_m_s": 0},
            {"maximum_speed_m_s": 4},
            {"maximum_elevation_rad": math.nan},
            {"maximum_aim_offset_rad": 0},
            {"nominal_spin_rad_s": 0},
            {"orientation_wxyz": (2.0, 0.0, 0.0, 0.0)},
            {"provenance": ""},
        )
        for override in invalid_profiles:
            with self.subTest(override=override), self.assertRaises(ValueError):
                ThrowProfile(
                    throw_type=ThrowType.BACKHAND,
                    **override,
                )


if __name__ == "__main__":
    unittest.main()
