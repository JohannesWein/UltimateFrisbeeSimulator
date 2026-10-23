import math
import unittest
from unittest.mock import patch

from ultimate_frisbee_simulator import (
    AerodynamicCoefficientModel,
    AutomaticTimingSettings,
    CoefficientSurface,
    FlightEnvironment,
    FlightInitialState,
    FlightTerminationReason,
    PlannedPass,
    NominalReleaseResult,
    ReleaseFailure,
    SolverSettings,
    ThrowProfile,
    ThrowType,
    TargetPassRequest,
    discraft_ultra_star,
    simulate_flight,
    plan_target_pass,
    solve_nominal_release,
)


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
        provenance="Zero coefficients for nominal solver tests",
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


class NominalReleaseTests(unittest.TestCase):
    def setUp(self):
        self.disc = zero_aerodynamic_disc()
        self.profile = ThrowProfile(
            throw_type=ThrowType.BACKHAND,
            nominal_spin_rad_s=10.0,
        )
        self.settings = SolverSettings(maximum_duration_s=3.0)

    def test_finds_release_that_reaches_target_at_exact_catch_time(self):
        catch_time = 0.7
        known_launch = FlightInitialState(
            position_m=(10.0, 8.0, 1.5),
            velocity_m_s=(15.0, 0.0, 3.0),
            angular_velocity_body_rad_s=(0.0, 0.0, 10.0),
        )
        known_trajectory = simulate_flight(
            self.disc,
            known_launch,
            settings=self.settings,
            sample_times_s=(catch_time,),
        )
        target = next(
            sample.position_m
            for sample in known_trajectory.samples
            if sample.elapsed_s == catch_time
        )
        planned = PlannedPass(
            throw_origin_xy_m=(10.0, 8.0),
            receiver_location_xy_m=target[:2],
            nominal_catch_time_s=catch_time,
            catch_height_m=target[2],
            target_tolerance_m=0.02,
        )

        result = solve_nominal_release(planned, self.profile, self.disc, settings=self.settings)

        self.assertTrue(result.feasible, result.failure)
        self.assertLessEqual(result.target_error_m, planned.target_tolerance_m)
        self.assertIsNone(result.failure)
        self.assertIn(
            catch_time,
            [sample.elapsed_s for sample in result.trajectory.samples],
        )

    def test_reports_target_outside_release_constraints(self):
        constrained = ThrowProfile(
            throw_type=ThrowType.BACKHAND,
            minimum_speed_m_s=5.0,
            maximum_speed_m_s=6.0,
            nominal_spin_rad_s=10.0,
        )
        planned = PlannedPass(
            throw_origin_xy_m=(0.0, 0.0),
            receiver_location_xy_m=(50.0, 0.0),
            nominal_catch_time_s=0.5,
            catch_height_m=1.0,
            target_tolerance_m=0.2,
        )

        result = solve_nominal_release(planned, constrained, self.disc, settings=self.settings)

        self.assertFalse(result.feasible)
        self.assertIs(result.failure, ReleaseFailure.TARGET_OUT_OF_TOLERANCE)
        self.assertGreater(result.target_error_m, planned.target_tolerance_m)

    def test_reports_ground_contact_before_requested_catch_time(self):
        constrained = ThrowProfile(
            throw_type=ThrowType.BACKHAND,
            minimum_speed_m_s=5.0,
            maximum_speed_m_s=6.0,
            nominal_spin_rad_s=10.0,
        )
        planned = PlannedPass(
            throw_origin_xy_m=(0.0, 0.0),
            receiver_location_xy_m=(10.0, 0.0),
            nominal_catch_time_s=2.5,
        )

        result = solve_nominal_release(planned, constrained, self.disc, settings=self.settings)

        self.assertFalse(result.feasible)
        self.assertIs(result.failure, ReleaseFailure.GROUND_BEFORE_CATCH)
        self.assertIs(
            result.trajectory.termination_reason,
            FlightTerminationReason.GROUND_CONTACT,
        )
        self.assertIn("last candidate", result.diagnostic)
        self.assertIn("before the requested catch time", result.diagnostic)
        self.assertIn("does not prove", result.diagnostic)

    def test_reports_catch_time_beyond_simulation_limit(self):
        planned = PlannedPass(
            throw_origin_xy_m=(0.0, 0.0),
            receiver_location_xy_m=(10.0, 0.0),
            nominal_catch_time_s=4.0,
        )

        result = solve_nominal_release(planned, self.profile, self.disc, settings=self.settings)

        self.assertFalse(result.feasible)
        self.assertIs(
            result.failure,
            ReleaseFailure.CATCH_AFTER_SIMULATION_LIMIT,
        )


class AutomaticTimingTests(unittest.TestCase):
    def setUp(self):
        self.disc = zero_aerodynamic_disc()
        self.request = TargetPassRequest(
            (10.0, 8.0), (20.0, 8.0),
            ThrowProfile(ThrowType.BACKHAND, nominal_spin_rad_s=10.0),
            target_tolerance_m=0.02,
        )
        self.timing = AutomaticTimingSettings(
            minimum_catch_time_s=0.4,
            maximum_catch_time_s=1.0,
            candidate_count=3,
            maximum_evaluations_per_candidate=6,
        )

    def test_real_search_is_repeatable_and_verifies_exact_time(self):
        first = plan_target_pass(self.request, self.disc, timing=self.timing)
        second = plan_target_pass(self.request, self.disc, timing=self.timing)
        self.assertEqual(first, second)
        self.assertTrue(first.feasible)
        plan = first.planned_pass
        sample = next(
            sample for sample in first.release_result.trajectory.samples
            if sample.elapsed_s == plan.nominal_catch_time_s
        )
        self.assertLessEqual(
            math.dist(sample.position_m, plan.catch_target_m),
            plan.target_tolerance_m,
        )
        self.assertGreater(sample.position_m[2], 0)
        self.assertEqual(plan.receiver_location_xy_m, self.request.receiver_location_xy_m)
        self.assertGreaterEqual(plan.nominal_catch_time_s, self.timing.minimum_catch_time_s)
        self.assertLessEqual(plan.nominal_catch_time_s, self.timing.maximum_catch_time_s)

    def test_search_stops_at_first_verified_candidate_in_ascending_order(self):
        failed = NominalReleaseResult(None, None, None, ReleaseFailure.GROUND_BEFORE_CATCH, 6)
        verified = solve_nominal_release(
            self.request.at_catch_time(0.7), self.request.throw_profile, self.disc,
        )
        self.assertTrue(verified.feasible)
        with patch(
            "ultimate_frisbee_simulator.throw_evaluation.solve_nominal_release",
            side_effect=(failed, verified),
        ) as solve:
            result = plan_target_pass(self.request, self.disc, timing=self.timing)
        self.assertEqual(result.attempted_catch_times_s, (0.4, 0.7))
        self.assertEqual(result.planned_pass.nominal_catch_time_s, 0.7)
        self.assertEqual(solve.call_count, 2)
        for call in solve.call_args_list:
            self.assertEqual(call.kwargs["maximum_evaluations"], 6)

    def test_exhausted_search_has_no_resolved_plan(self):
        failed = NominalReleaseResult(None, None, None, ReleaseFailure.OPTIMIZATION_FAILED, 6)
        with patch(
            "ultimate_frisbee_simulator.throw_evaluation.solve_nominal_release",
            return_value=failed,
        ) as solve:
            result = plan_target_pass(self.request, self.disc, timing=self.timing)
        self.assertFalse(result.feasible)
        self.assertIsNone(result.planned_pass)
        self.assertEqual(result.attempted_catch_times_s, (0.4, 0.7, 1.0))
        self.assertEqual(solve.call_count, self.timing.candidate_count)
        self.assertIs(result.release_result, failed)
        self.assertIn("search bounds and budget", result.diagnostic)
        self.assertIn("does not prove", result.diagnostic)

    def test_solver_budget_bounds_actual_flight_integrations(self):
        with patch(
            "ultimate_frisbee_simulator.throw_evaluation.simulate_flight",
            wraps=simulate_flight,
        ) as simulate:
            result = solve_nominal_release(
                self.request.at_catch_time(0.1), self.request.throw_profile,
                self.disc, maximum_evaluations=2,
            )
        self.assertLessEqual(result.optimizer_evaluations, 2)
        self.assertLessEqual(simulate.call_count, 4 * 2 + 1)

    def test_rejects_search_outside_integration_horizon(self):
        with self.assertRaisesRegex(ValueError, "simulation duration"):
            plan_target_pass(
                self.request, self.disc,
                settings=SolverSettings(maximum_duration_s=0.5),
                timing=self.timing,
            )

    def test_integration_errors_are_not_hidden_as_search_failures(self):
        with patch(
            "ultimate_frisbee_simulator.throw_evaluation.solve_nominal_release",
            side_effect=RuntimeError("integration failed"),
        ), self.assertRaisesRegex(RuntimeError, "integration failed"):
            plan_target_pass(self.request, self.disc, timing=self.timing)


if __name__ == "__main__":
    unittest.main()
