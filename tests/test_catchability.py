import math
import unittest
from dataclasses import asdict

import numpy as np

from ultimate_frisbee_simulator import (
    AerodynamicCoefficientModel,
    AutomaticTimingSettings,
    CatchabilityResult,
    CoefficientSurface,
    FlightInitialState,
    PlannedPass,
    SolverSettings,
    TargetPassRequest,
    ThrowEvaluationError,
    ThrowProfile,
    ThrowType,
    ThrowVariationProfile,
    discraft_ultra_star,
    evaluate_flight_catchability,
    plan_target_pass,
    sample_throw_execution,
    simulate_flight,
    solve_nominal_release,
)
from ultimate_frisbee_simulator.simulation import FlightTrajectory, TrajectorySample
from ultimate_frisbee_simulator import FlightTerminationReason, NominalReleaseResult


def zero_aerodynamic_disc():
    disc = discraft_ultra_star()
    axis = (-math.pi, math.pi)
    surface = CoefficientSurface(
        angle_of_attack_rad=axis,
        sideslip_rad=axis,
        values=((0.0, 0.0), (0.0, 0.0)),
    )
    model = AerodynamicCoefficientModel(
        lift=surface,
        drag=surface,
        side_force=surface,
        roll_moment=surface,
        pitch_moment=surface,
        yaw_moment=surface,
        provenance="Zero coefficients for catchability tests",
    )
    return type(disc)(
        name=disc.name,
        mass_kg=disc.mass_kg,
        diameter_m=disc.diameter_m,
        reference_area_m2=disc.reference_area_m2,
        principal_moments_kg_m2=disc.principal_moments_kg_m2,
        parameter_sources=disc.parameter_sources,
        aerodynamic_coefficients=model,
    )


def make_plan_and_nominal():
    disc = zero_aerodynamic_disc()
    profile = ThrowProfile(
        throw_type=ThrowType.BACKHAND,
        nominal_spin_rad_s=10.0,
    )
    launch = FlightInitialState(
        position_m=(0.0, 0.0, 1.5),
        velocity_m_s=(15.0, 0.0, 3.0),
        angular_velocity_body_rad_s=(0.0, 0.0, 10.0),
    )
    catch_time = 0.7
    trajectory = simulate_flight(
        disc,
        launch,
        settings=SolverSettings(maximum_duration_s=1.0),
        sample_times_s=(catch_time,),
    )
    catch_sample = next(
        sample for sample in trajectory.samples if sample.elapsed_s == catch_time
    )
    planned = PlannedPass(
        throw_origin_xy_m=(0.0, 0.0),
        receiver_location_xy_m=catch_sample.position_m[:2],
        nominal_catch_time_s=catch_time,
        catch_height_m=catch_sample.position_m[2],
        catch_radius_m=1.0,
        catch_time_tolerance_s=0.1,
    )
    nominal = NominalReleaseResult(
        launch,
        trajectory,
        0.0,
        None,
        1,
    )
    return disc, profile, planned, nominal


def variation_profile(
    *,
    speed_stddev=0.05,
    direction_stddev=0.002,
    elevation_stddev=0.002,
    spin_stddev=0.1,
):
    return ThrowVariationProfile(
        throw_type=ThrowType.BACKHAND,
        speed_stddev_m_s=speed_stddev,
        speed_stddev_increase_per_m_m_s=0.0,
        direction_stddev_rad=direction_stddev,
        direction_stddev_increase_per_m_rad=0.0,
        elevation_stddev_rad=elevation_stddev,
        elevation_stddev_increase_per_m_rad=0.0,
        spin_stddev_rad_s=spin_stddev,
        spin_stddev_increase_per_m_rad_s=0.0,
    )


class FlightCatchabilityTests(unittest.TestCase):
    def setUp(self):
        (
            self.disc,
            self.throw_profile,
            self.planned,
            self.nominal,
        ) = make_plan_and_nominal()
        self.variation = variation_profile()

    def evaluate(self, seed, planned=None, nominal=None):
        return evaluate_flight_catchability(
            planned or self.planned,
            nominal or self.nominal,
            self.throw_profile,
            self.variation,
            self.disc,
            np.random.default_rng(seed),
            trial_count=8,
            settings=SolverSettings(maximum_duration_s=1.0),
        )

    def test_seed_repeats_trial_outcomes_and_provenance(self):
        first = self.evaluate(2026)
        second = self.evaluate(2026)

        self.assertEqual(first, second)
        self.assertEqual(first.total_trial_count, 8)
        self.assertEqual(
            first.successful_trial_count,
            sum(first.trial_successes),
        )
        self.assertEqual(
            first.catchability_fraction,
            first.successful_trial_count / 8,
        )
        self.assertEqual(
            first.flight_model_provenance,
            self.disc.aerodynamic_coefficients.provenance,
        )
        self.assertEqual(first.throw_variation_provenance, self.variation.provenance)
        self.assertIn("not full pass-completion", first.interpretation)

    def test_spatial_catch_window_rejects_distant_target(self):
        distant_plan = PlannedPass(
            throw_origin_xy_m=(0.0, 0.0),
            receiver_location_xy_m=(40.0, 30.0),
            nominal_catch_time_s=self.planned.nominal_catch_time_s,
            catch_height_m=self.planned.catch_height_m,
            catch_radius_m=0.1,
            catch_time_tolerance_s=0.01,
        )

        result = self.evaluate(4, planned=distant_plan)

        self.assertEqual(result.successful_trial_count, 0)
        self.assertEqual(result.trial_successes, (False,) * 8)

    def test_reports_invalid_trial_with_its_number(self):
        invalid_launch = FlightInitialState(
            position_m=(0.0, 0.0, 1.5),
            velocity_m_s=(100.0, 0.0, 0.0),
            angular_velocity_body_rad_s=(0.0, 0.0, 10.0),
        )
        sample = TrajectorySample(
            elapsed_s=0.0,
            position_m=invalid_launch.position_m,
            velocity_m_s=invalid_launch.velocity_m_s,
            orientation_wxyz=invalid_launch.orientation_wxyz,
            angular_velocity_body_rad_s=invalid_launch.angular_velocity_body_rad_s,
        )
        invalid_nominal = NominalReleaseResult(
            invalid_launch,
            FlightTrajectory((sample,), FlightTerminationReason.MAXIMUM_DURATION),
            None,
            None,
            1,
        )

        with self.assertRaisesRegex(ThrowEvaluationError, "trial 1 failed"):
            self.evaluate(4, nominal=invalid_nominal)

    def test_rejects_invalid_trial_count_and_step(self):
        for kwargs in (
            {"trial_count": 0},
            {"trial_count": True},
            {"time_step_s": 0.0},
            {"time_step_s": math.nan},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                evaluate_flight_catchability(
                    self.planned,
                    self.nominal,
                    self.throw_profile,
                    self.variation,
                    self.disc,
                    np.random.default_rng(1),
                    **kwargs,
                )

    def test_catchability_result_rejects_empty_or_invalid_outcomes(self):
        with self.assertRaisesRegex(ValueError, "trial_successes"):
            CatchabilityResult((), "flight model", "variation")
        with self.assertRaisesRegex(ValueError, "trial_successes"):
            CatchabilityResult((True, 1), "flight model", "variation")

    def test_both_planning_modes_preserve_resolved_plan_during_execution(self):
        request = TargetPassRequest(
            (0.0, 0.0), (10.0, 0.0), self.throw_profile,
        )
        automatic = plan_target_pass(
            request, self.disc,
            timing=AutomaticTimingSettings(
                minimum_catch_time_s=0.4, maximum_catch_time_s=1.0,
                candidate_count=3,
            ),
        )
        timed_plan = request.at_catch_time(0.7)
        timed_release = solve_nominal_release(
            timed_plan, self.throw_profile, self.disc,
        )
        for plan, release in (
            (automatic.planned_pass, automatic.release_result),
            (timed_plan, timed_release),
        ):
            with self.subTest(catch_time=plan.nominal_catch_time_s):
                self.assertTrue(release.feasible)
                original = asdict(plan)
                first = sample_throw_execution(
                    plan, release, self.throw_profile, self.variation,
                    np.random.default_rng(42),
                )
                second = sample_throw_execution(
                    plan, release, self.throw_profile, self.variation,
                    np.random.default_rng(42),
                )
                self.assertEqual(first, second)
                self.assertEqual(
                    self.evaluate(42, planned=plan, nominal=release),
                    self.evaluate(42, planned=plan, nominal=release),
                )
                self.assertEqual(asdict(plan), original)
                self.assertEqual(plan.receiver_location_xy_m, request.receiver_location_xy_m)

    def test_unresolved_automatic_plan_cannot_be_sampled_or_evaluated(self):
        request = TargetPassRequest(
            (0.0, 0.0), (1000.0, 0.0), self.throw_profile,
        )
        result = plan_target_pass(
            request, self.disc,
            timing=AutomaticTimingSettings(
                minimum_catch_time_s=0.1, maximum_catch_time_s=0.2,
                candidate_count=2, maximum_evaluations_per_candidate=2,
            ),
        )
        self.assertIsNone(result.planned_pass)
        for execute in (sample_throw_execution, evaluate_flight_catchability):
            args = [
                result.planned_pass, result.release_result,
                self.throw_profile, self.variation,
            ]
            if execute is evaluate_flight_catchability:
                args.append(self.disc)
            args.append(np.random.default_rng(1))
            with self.subTest(execute=execute), self.assertRaisesRegex(
                ValueError, "feasible launch state",
            ):
                execute(*args)


if __name__ == "__main__":
    unittest.main()
