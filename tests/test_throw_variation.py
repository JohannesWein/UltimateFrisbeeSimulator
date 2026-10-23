import math
import unittest

import numpy as np

from ultimate_frisbee_simulator import (
    FlightInitialState,
    FlightTerminationReason,
    FlightTrajectory,
    NominalReleaseResult,
    PlannedPass,
    ThrowProfile,
    ThrowType,
    ThrowVariationProfile,
    default_throw_profiles,
    default_throw_variation_profiles,
    sample_throw_execution,
)
from ultimate_frisbee_simulator.simulation import TrajectorySample


def feasible_nominal_release():
    launch = FlightInitialState(
        position_m=(0.0, 0.0, 1.5),
        velocity_m_s=(14.0, 0.0, 2.0),
        angular_velocity_body_rad_s=(0.0, 0.0, 50.0),
    )
    sample = TrajectorySample(
        elapsed_s=0.5,
        position_m=(7.0, 0.0, 1.0),
        velocity_m_s=launch.velocity_m_s,
        orientation_wxyz=launch.orientation_wxyz,
        angular_velocity_body_rad_s=launch.angular_velocity_body_rad_s,
    )
    return NominalReleaseResult(
        launch,
        FlightTrajectory((sample,), FlightTerminationReason.MAXIMUM_DURATION),
        0.0,
        None,
        1,
    )


class ThrowVariationProfileTests(unittest.TestCase):
    def test_spreads_increase_with_throw_distance(self):
        profiles = default_throw_variation_profiles()
        backhand = profiles[ThrowType.BACKHAND]
        forehand = profiles[ThrowType.FOREHAND]

        self.assertTrue(
            all(
                long_distance > short_distance
                for short_distance, long_distance in zip(
                    backhand.standard_deviations(5.0),
                    backhand.standard_deviations(20.0),
                )
            )
        )
        self.assertNotEqual(
            backhand.standard_deviations(10.0),
            forehand.standard_deviations(10.0),
        )

    def test_rejects_invalid_spreads_and_distances(self):
        with self.assertRaises(ValueError):
            ThrowVariationProfile(
                throw_type=ThrowType.BACKHAND,
                speed_stddev_m_s=-0.1,
                speed_stddev_increase_per_m_m_s=0.0,
                direction_stddev_rad=0.0,
                direction_stddev_increase_per_m_rad=0.0,
                elevation_stddev_rad=0.0,
                elevation_stddev_increase_per_m_rad=0.0,
                spin_stddev_rad_s=0.0,
                spin_stddev_increase_per_m_rad_s=0.0,
            )
        with self.assertRaisesRegex(ValueError, "distance_m"):
            default_throw_variation_profiles()[
                ThrowType.BACKHAND
            ].standard_deviations(math.inf)


class ThrowExecutionSamplingTests(unittest.TestCase):
    def setUp(self):
        self.plan = PlannedPass(
            throw_origin_xy_m=(0.0, 0.0),
            receiver_location_xy_m=(10.0, 0.0),
            nominal_catch_time_s=0.5,
        )
        self.throw_profile = default_throw_profiles()[ThrowType.BACKHAND]
        self.variation_profile = default_throw_variation_profiles()[
            ThrowType.BACKHAND
        ]
        self.nominal = feasible_nominal_release()

    def sample(self, seed):
        return sample_throw_execution(
            self.plan,
            self.nominal,
            self.throw_profile,
            self.variation_profile,
            np.random.default_rng(seed),
        )

    def test_same_generator_seed_repeats_execution(self):
        self.assertEqual(self.sample(1234), self.sample(1234))

    def test_samples_stay_within_release_constraints(self):
        generator = np.random.default_rng(5)
        for _ in range(100):
            launch = sample_throw_execution(
                self.plan,
                self.nominal,
                self.throw_profile,
                self.variation_profile,
                generator,
            )
            speed = math.sqrt(sum(component**2 for component in launch.velocity_m_s))
            horizontal_speed = math.hypot(*launch.velocity_m_s[:2])
            heading = math.atan2(launch.velocity_m_s[1], launch.velocity_m_s[0])
            aim_offset = math.atan2(math.sin(heading), math.cos(heading))
            elevation = math.atan2(launch.velocity_m_s[2], horizontal_speed)

            self.assertGreaterEqual(speed, self.throw_profile.minimum_speed_m_s)
            self.assertLessEqual(speed, self.throw_profile.maximum_speed_m_s)
            self.assertLessEqual(
                abs(aim_offset),
                self.throw_profile.maximum_aim_offset_rad,
            )
            self.assertGreaterEqual(
                elevation,
                self.throw_profile.minimum_elevation_rad,
            )
            self.assertLessEqual(
                elevation,
                self.throw_profile.maximum_elevation_rad,
            )
            self.assertGreater(launch.angular_velocity_body_rad_s[2], 0)

    def test_rejects_profile_mismatch_and_invalid_random_generator(self):
        forehand_variation = default_throw_variation_profiles()[
            ThrowType.FOREHAND
        ]
        with self.assertRaisesRegex(ValueError, "same throw_type"):
            sample_throw_execution(
                self.plan,
                self.nominal,
                self.throw_profile,
                forehand_variation,
                np.random.default_rng(1),
            )
        with self.assertRaisesRegex(TypeError, "Generator"):
            sample_throw_execution(
                self.plan,
                self.nominal,
                self.throw_profile,
                self.variation_profile,
                np.random.RandomState(1),
            )

    def test_rejects_infeasible_nominal_release(self):
        infeasible = NominalReleaseResult(
            None,
            None,
            None,
            None,
            0,
        )
        with self.assertRaisesRegex(ValueError, "feasible launch"):
            sample_throw_execution(
                self.plan,
                infeasible,
                self.throw_profile,
                self.variation_profile,
                np.random.default_rng(1),
            )


if __name__ == "__main__":
    unittest.main()
