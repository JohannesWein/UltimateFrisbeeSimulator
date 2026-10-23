import unittest
from unittest.mock import patch
from time import perf_counter

import numpy as np

import matplotlib
matplotlib.use("Agg")

from matplotlib.backend_bases import KeyEvent, MouseEvent

from ultimate_frisbee_simulator import (
    AutomaticTimingSettings,
    FieldGeometry,
    PassPlanningMode,
    ReleaseFailure,
    ThrowType,
    create_scenario_editor,
    default_throw_variation_profiles,
    plan_target_pass,
    sample_throw_execution,
)
from tests.test_throw_evaluation import zero_aerodynamic_disc


class ScenarioEditorInteractionTests(unittest.TestCase):
    def setUp(self):
        self.editor = create_scenario_editor(trial_count=2)
        self.editor.render()

    def tearDown(self):
        self.editor.close()

    def click_field(self, x_m, y_m):
        pixel_x, pixel_y = self.editor.field_axes.transData.transform((x_m, y_m))
        event = MouseEvent(
            "button_press_event",
            self.editor.figure.canvas,
            pixel_x,
            pixel_y,
            button=1,
        )
        self.editor.figure.canvas.callbacks.process("button_press_event", event)

    def test_mouse_clicks_define_field_coordinates(self):
        self.click_field(20.0, 8.5)
        self.assertIsNone(self.editor.planned_pass)
        self.click_field(35.0, 24.0)

        self.assertIsNone(self.editor.planned_pass)
        self.assertFalse(self.editor.catch_time_widget.active)
        self.assertFalse(self.editor.time_axes.get_visible())
        self.assertAlmostEqual(
            self.editor.target_request.throw_origin_xy_m[0],
            20.0,
        )
        self.assertAlmostEqual(
            self.editor.target_request.throw_origin_xy_m[1],
            8.5,
        )
        self.assertAlmostEqual(
            self.editor.target_request.receiver_location_xy_m[0],
            35.0,
        )
        self.assertAlmostEqual(
            self.editor.target_request.receiver_location_xy_m[1],
            24.0,
        )

    def test_time_and_throw_profile_widgets_update_the_plan(self):
        self.click_field(20.0, 8.5)
        self.click_field(35.0, 24.0)

        self.editor.mode_widget.set_active(1)
        self.assertTrue(self.editor.catch_time_widget.active)
        self.assertTrue(self.editor.time_axes.get_visible())
        self.editor.catch_time_widget.begin_typing()
        self.editor.catch_time_widget.set_val("0.8")
        event = KeyEvent(
            "key_press_event",
            self.editor.figure.canvas,
            key="enter",
        )
        self.editor.figure.canvas.callbacks.process("key_press_event", event)
        self.editor.profile_widget.set_active(1)

        self.assertEqual(self.editor.planned_pass.nominal_catch_time_s, 0.8)
        self.assertIs(self.editor.throw_type, ThrowType.FOREHAND)

    def test_custom_field_dimensions_are_reflected_in_axes(self):
        self.editor.close()
        self.editor = create_scenario_editor(
            field=FieldGeometry(
                length_m=110.0,
                width_m=40.0,
                end_zone_depth_m=18.0,
            ),
            trial_count=1,
        )

        self.assertEqual(self.editor.field_axes.get_xlim(), (0.0, 110.0))
        self.assertEqual(self.editor.field_axes.get_ylim(), (0.0, 40.0))

    def test_end_to_end_pass_plan_solver_estimate_and_render(self):
        self.click_field(20.0, 18.5)
        self.click_field(26.1845, 18.3099)
        self.editor.mode_widget.set_active(1)
        self.editor.catch_time_widget.begin_typing()
        self.editor.catch_time_widget.set_val("0.7")
        event = KeyEvent(
            "key_press_event",
            self.editor.figure.canvas,
            key="enter",
        )
        self.editor.figure.canvas.callbacks.process("key_press_event", event)

        planned_before_evaluation = self.editor.planned_pass
        self.editor.evaluate()
        self.editor.render()

        self.assertEqual(self.editor.planned_pass, planned_before_evaluation)
        self.assertTrue(self.editor.release_result.feasible)
        self.assertLessEqual(
            self.editor.release_result.target_error_m,
            planned_before_evaluation.target_tolerance_m,
        )
        self.assertIsNotNone(self.editor.catchability_result)
        self.assertEqual(
            self.editor.catchability_result.total_trial_count,
            self.editor.trial_count,
        )
        self.assertGreater(len(self.editor.trajectory_artist.get_xdata()), 1)
        self.assertIn("Flight-level catchability", self.editor.status_artist.get_text())

    def test_end_to_end_automatic_pass_with_default_flight_model(self):
        self.click_field(20.0, 18.5)
        self.click_field(26.0, 18.5)
        request = self.editor.target_request

        self.editor.evaluate()
        self.editor.render()

        planned = self.editor.planned_pass
        self.assertIsNotNone(planned, self.editor.status_artist.get_text())
        self.assertTrue(self.editor.release_result.feasible)
        self.assertEqual(self.editor.target_request, request)
        self.assertEqual(planned.receiver_location_xy_m, request.receiver_location_xy_m)
        self.assertGreater(planned.nominal_catch_time_s, 0)
        self.assertLessEqual(
            self.editor.release_result.target_error_m, planned.target_tolerance_m,
        )
        self.assertEqual(
            self.editor.catchability_result.total_trial_count, self.editor.trial_count,
        )
        self.assertIn("(automatic)", self.editor.catch_time_artist.get_text())
        self.assertGreater(len(self.editor.trajectory_artist.get_xdata()), 1)
        self.assertIn("Flight-level catchability", self.editor.status_artist.get_text())

    def test_reported_diagonal_pass_uses_verified_immutable_plan(self):
        self.click_field(36.5, 17.0)
        self.click_field(63.0, 26.5)
        request = self.editor.target_request
        results = []

        def record_plan(*args, **kwargs):
            result = plan_target_pass(*args, **kwargs)
            results.append(result)
            return result

        start = perf_counter()
        with patch(
            "ultimate_frisbee_simulator.scenario_editor.plan_target_pass",
            side_effect=record_plan,
        ) as planner:
            self.editor.evaluate()
        elapsed = perf_counter() - start
        planner.assert_called_once_with(request, self.editor.disc, timing=self.editor.timing)
        planned = self.editor.planned_pass
        self.assertIsNotNone(planned, self.editor.status_artist.get_text())
        release = self.editor.release_result
        sample = next(
            sample for sample in release.trajectory.samples
            if sample.elapsed_s == planned.nominal_catch_time_s
        )
        self.assertLessEqual(
            np.linalg.norm(np.asarray(sample.position_m) - planned.catch_target_m), 0.5,
        )
        self.assertTrue(all(sample.position_m[2] > 0 for sample in release.trajectory.samples))
        profile = request.throw_profile
        variation = default_throw_variation_profiles()[ThrowType.BACKHAND]
        first = sample_throw_execution(
            planned, release, profile, variation, np.random.default_rng(42),
        )
        second = sample_throw_execution(
            planned, release, profile, variation, np.random.default_rng(42),
        )
        self.assertEqual(first, second)
        self.assertIs(self.editor.planned_pass, planned)
        self.assertEqual(self.editor.target_request, request)
        self.assertEqual(planned.receiver_location_xy_m, request.receiver_location_xy_m)
        self.assertEqual(self.editor.catchability_result.total_trial_count, 2)
        self.editor.render()
        self.assertGreater(len(self.editor.trajectory_artist.get_xdata()), 1)
        attempted = results[0].attempted_catch_times_s
        print(f"Reported pass: catch={planned.nominal_catch_time_s:.3f}s, "
              f"attempted={attempted}, evaluation={elapsed:.2f}s")

    def test_infeasible_plan_is_visibly_identified(self):
        self.click_field(20.0, 18.5)
        self.click_field(90.0, 36.0)
        self.editor.mode_widget.set_active(1)
        self.editor.catch_time_widget.begin_typing()
        self.editor.catch_time_widget.set_val("0.1")
        event = KeyEvent(
            "key_press_event",
            self.editor.figure.canvas,
            key="enter",
        )
        self.editor.figure.canvas.callbacks.process("key_press_event", event)

        self.editor.evaluate()
        self.editor.render()

        self.assertFalse(self.editor.release_result.feasible)
        self.assertIsNone(self.editor.catchability_result)
        self.assertEqual(self.editor.status_artist.get_color(), "#b22222")
        status = " ".join(self.editor.status_artist.get_text().split())
        self.assertIn("No suitable nominal release found", status)
        self.assertIn("does not prove", status)

    def use_simple_physics(self):
        self.editor.close()
        self.editor = create_scenario_editor(
            disc=zero_aerodynamic_disc(),
            trial_count=2,
            timing=AutomaticTimingSettings(
                minimum_catch_time_s=0.4, maximum_catch_time_s=1.0,
                candidate_count=3,
            ),
        )
        self.editor.render()
        self.click_field(20, 18.5)
        self.click_field(30, 18.5)

    def test_automatic_mode_displays_resolved_time_and_plan(self):
        self.use_simple_physics()
        self.assertIs(self.editor.planning_mode, PassPlanningMode.TARGET_ONLY)
        self.assertIsNone(self.editor.planned_pass)
        self.editor.evaluate()
        self.editor.render()
        self.assertTrue(self.editor.release_result.feasible)
        self.assertIsNotNone(self.editor.planned_pass)
        self.assertIn(
            f"{self.editor.planned_pass.nominal_catch_time_s:.3f} s",
            self.editor.catch_time_artist.get_text(),
        )
        self.assertTrue(self.editor.catch_time_artist.get_visible())
        self.assertFalse(self.editor.time_axes.get_visible())
        self.assertEqual(len(self.editor.plan_artist.get_xdata()), 2)
        self.assertGreater(len(self.editor.trajectory_artist.get_xdata()), 1)

    def test_mode_and_input_changes_clear_stale_results(self):
        for edit in (
            lambda: self.editor.mode_widget.set_active(1),
            lambda: self.editor.profile_widget.set_active(1),
            lambda: self.click_field(50, 10),
        ):
            with self.subTest(edit=edit):
                self.use_simple_physics()
                self.editor.evaluate()
                self.assertIsNotNone(self.editor.catchability_result)
                edit()
                self.assertIsNone(self.editor.release_result)
                self.assertIsNone(self.editor.catchability_result)
                self.assertEqual(len(self.editor.trajectory_artist.get_xdata()), 0)
                if self.editor.planning_mode is PassPlanningMode.TARGET_ONLY:
                    self.assertIsNone(self.editor.planned_pass)
        self.editor.set_planning_mode(PassPlanningMode.TIMING_PASS)
        self.editor.set_catch_time(0.7)
        self.editor.evaluate()
        self.editor.set_catch_time(0.8)
        self.assertIsNone(self.editor.release_result)
        self.assertIsNone(self.editor.catchability_result)
        self.editor.mode_widget.set_active(0)
        self.assertIsNone(self.editor.planned_pass)
        self.assertFalse(self.editor.catch_time_widget.active)

    def test_invalid_timing_text_cannot_reuse_previous_valid_time(self):
        self.use_simple_physics()
        self.editor.mode_widget.set_active(1)
        self.editor.set_catch_time(0.7)
        self.editor.evaluate()
        self.editor.catch_time_widget.set_val("not a number")
        with patch("ultimate_frisbee_simulator.scenario_editor.solve_nominal_release") as solve:
            self.editor.evaluate()
        solve.assert_not_called()
        self.assertIsNone(self.editor.planned_pass)
        self.assertIsNone(self.editor.catchability_result)
        self.assertEqual(self.editor.status_artist.get_color(), "#b22222")
        self.assertIn("Invalid catch time", self.editor.status_artist.get_text())

    def test_timing_mode_does_not_substitute_an_automatic_time(self):
        self.use_simple_physics()
        self.editor.mode_widget.set_active(1)
        self.editor.set_catch_time(11.0)
        with patch("ultimate_frisbee_simulator.scenario_editor.plan_target_pass") as auto:
            self.editor.evaluate()
        auto.assert_not_called()
        self.assertEqual(self.editor.planned_pass.nominal_catch_time_s, 11.0)
        self.assertIs(
            self.editor.release_result.failure, ReleaseFailure.CATCH_AFTER_SIMULATION_LIMIT,
        )
        self.assertIsNone(self.editor.catchability_result)

    def test_failed_automatic_search_never_runs_catchability(self):
        self.use_simple_physics()
        self.editor.timing = AutomaticTimingSettings(
            minimum_catch_time_s=0.01, maximum_catch_time_s=0.02,
            candidate_count=2, maximum_evaluations_per_candidate=2,
        )
        with patch(
            "ultimate_frisbee_simulator.scenario_editor.evaluate_flight_catchability",
        ) as evaluate:
            self.editor.evaluate()
        evaluate.assert_not_called()
        self.assertIsNone(self.editor.planned_pass)
        self.assertFalse(self.editor.release_result.feasible)
        self.assertIsNone(self.editor.catchability_result)
        status = " ".join(self.editor.status_artist.get_text().split())
        self.assertIn("search bounds and budget", status)
        self.assertIn("does not prove", status)
        self.assertEqual(self.editor.status_artist.get_color(), "#b22222")
        self.editor.render()


if __name__ == "__main__":
    unittest.main()
