"""Local Matplotlib editor for tactical pass scenarios."""

import math
from enum import Enum
from numbers import Integral, Real
import textwrap

from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.widgets import Button, RadioButtons, TextBox
import numpy as np

from .catchability import (
    CatchabilityResult,
    evaluate_flight_catchability,
)
from .field import FieldGeometry
from .flight import DiscProfile, discraft_ultra_star
from .throw_evaluation import (
    AutomaticTimingSettings,
    NominalReleaseResult,
    plan_target_pass,
    solve_nominal_release,
)
from .throwing import PlannedPass, TargetPassRequest, ThrowType, default_throw_profiles
from .throw_variation import (
    default_throw_variation_profiles,
)


class PassPlanningMode(Enum):
    TARGET_ONLY = "Pass to target"
    TIMING_PASS = "Timing pass"


class ScenarioEditor:
    """Interactive, local top-down field editor and pass evaluator."""

    def __init__(
        self,
        field: FieldGeometry = FieldGeometry(),
        disc: DiscProfile | None = None,
        *,
        trial_count: int = 20,
        random_seed: int = 42,
        timing: AutomaticTimingSettings = AutomaticTimingSettings(),
    ) -> None:
        if (
            isinstance(trial_count, bool)
            or not isinstance(trial_count, Integral)
            or trial_count < 1
        ):
            raise ValueError("trial_count must be a positive integer")
        if isinstance(random_seed, bool) or not isinstance(random_seed, Integral):
            raise ValueError("random_seed must be an integer")
        self.field = field
        self.disc = disc or discraft_ultra_star()
        self.trial_count = trial_count
        self.random_seed = random_seed
        self.timing = timing
        self.planning_mode = PassPlanningMode.TARGET_ONLY
        self._catch_time_s = 1.5
        self._time_input_error: str | None = None
        self._resolved_pass: PlannedPass | None = None
        self._throw_origin_xy_m: tuple[float, float] | None = None
        self._receiver_location_xy_m: tuple[float, float] | None = None
        self.throw_profiles = default_throw_profiles()
        self.variation_profiles = default_throw_variation_profiles()
        self.throw_type = ThrowType.BACKHAND
        self.release_result: NominalReleaseResult | None = None
        self.catchability_result: CatchabilityResult | None = None
        self.analysis_error: str | None = None

        import matplotlib.pyplot as plt

        self.figure: Figure = plt.figure(figsize=(12, 8))
        self.field_axes: Axes = self.figure.add_axes((0.06, 0.12, 0.63, 0.8))
        self._draw_field()
        self.origin_artist, = self.field_axes.plot(
            [], [], marker="o", color="#165d9c", markersize=8, linestyle="None",
            label="Thrower",
        )
        self.receiver_artist, = self.field_axes.plot(
            [], [], marker="^", color="#e4572e", markersize=8, linestyle="None",
            label="Receiver target",
        )
        self.plan_artist, = self.field_axes.plot(
            [], [], color="#343a40", linewidth=1.8, linestyle="--",
            label="Planned pass",
        )
        self.trajectory_artist, = self.field_axes.plot(
            [], [], color="#e4572e", linewidth=2.0, label="Nominal flight",
        )
        self.field_axes.legend(loc="upper left")

        self.mode_axes = self.figure.add_axes((0.73, 0.79, 0.25, 0.13))
        self.mode_widget = RadioButtons(
            self.mode_axes,
            tuple(mode.value for mode in PassPlanningMode),
            active=0,
        )
        self.mode_widget.on_clicked(self._on_planning_mode_selected)
        self.profile_axes = self.figure.add_axes((0.73, 0.63, 0.25, 0.13))
        self.profile_widget = RadioButtons(
            self.profile_axes,
            (ThrowType.BACKHAND.value, ThrowType.FOREHAND.value),
            active=0,
        )
        self.profile_widget.on_clicked(self._on_throw_type_selected)

        self.time_axes = self.figure.add_axes((0.83, 0.55, 0.15, 0.05))
        self.catch_time_widget = TextBox(
            self.time_axes,
            "Catch time (s)",
            initial=str(self._catch_time_s),
        )
        self.catch_time_widget.on_submit(self._on_catch_time_submitted)
        self.catch_time_widget.set_active(False)
        self.time_axes.set_visible(False)
        self.catch_time_artist = self.figure.text(
            0.73, 0.57, "Catch time: determined automatically", fontsize=9,
        )

        self.evaluate_axes = self.figure.add_axes((0.73, 0.46, 0.25, 0.06))
        self.evaluate_widget = Button(self.evaluate_axes, "Evaluate pass")
        self.evaluate_widget.on_clicked(self._on_evaluate_clicked)
        self.status_artist = self.figure.text(
            0.73,
            0.42,
            "Click the field to place thrower, then receiver.",
            ha="left",
            va="top",
            wrap=True,
            fontsize=9,
        )
        self.figure.canvas.mpl_connect("button_press_event", self._on_field_click)
        self._refresh_positions()

    @property
    def target_request(self) -> TargetPassRequest | None:
        if (
            self._throw_origin_xy_m is None
            or self._receiver_location_xy_m is None
        ):
            return None
        return TargetPassRequest(
            throw_origin_xy_m=self._throw_origin_xy_m,
            receiver_location_xy_m=self._receiver_location_xy_m,
            throw_profile=self.throw_profiles[self.throw_type],
        )

    @property
    def planned_pass(self) -> PlannedPass | None:
        if self.planning_mode is PassPlanningMode.TARGET_ONLY:
            return self._resolved_pass
        request = self.target_request
        if request is None or self._time_input_error is not None:
            return None
        return request.at_catch_time(self._catch_time_s)

    def _draw_field(self) -> None:
        self.field_axes.set_xlim(0.0, self.field.length_m)
        self.field_axes.set_ylim(0.0, self.field.width_m)
        self.field_axes.set_aspect("equal", adjustable="box")
        self.field_axes.set_xlabel("Field length x (m)")
        self.field_axes.set_ylabel("Field width y (m)")
        self.field_axes.set_title("Ultimate Frisbee pass scenario")
        self.field_axes.set_facecolor("#78a96b")
        self.field_axes.axvspan(
            0.0,
            self.field.end_zone_depth_m,
            color="#668b5a",
            alpha=0.7,
        )
        self.field_axes.axvspan(
            self.field.length_m - self.field.end_zone_depth_m,
            self.field.length_m,
            color="#668b5a",
            alpha=0.7,
        )
        self.field_axes.axvline(
            self.field.end_zone_depth_m,
            color="white",
            linewidth=1.5,
        )
        self.field_axes.axvline(
            self.field.length_m - self.field.end_zone_depth_m,
            color="white",
            linewidth=1.5,
        )
        self.field_axes.grid(color="white", alpha=0.3, linewidth=0.7)

    def _set_status(self, message: str, *, error: bool = False) -> None:
        self.status_artist.set_text(
            "\n".join(textwrap.fill(line, width=43) for line in message.splitlines())
        )
        self.status_artist.set_color("#b22222" if error else "#222222")
        self.figure.canvas.draw_idle()

    def _clear_analysis(self) -> None:
        self._resolved_pass = None
        self.release_result = None
        self.catchability_result = None
        self.analysis_error = None
        self.trajectory_artist.set_data([], [])
        self.catch_time_artist.set_text("Catch time: determined automatically")

    def _refresh_positions(self) -> None:
        if self._throw_origin_xy_m is None:
            self.origin_artist.set_data([], [])
        else:
            self.origin_artist.set_data(
                [self._throw_origin_xy_m[0]],
                [self._throw_origin_xy_m[1]],
            )
        if self._receiver_location_xy_m is None:
            self.receiver_artist.set_data([], [])
        else:
            self.receiver_artist.set_data(
                [self._receiver_location_xy_m[0]],
                [self._receiver_location_xy_m[1]],
            )
        request = self.target_request
        if request is None:
            self.plan_artist.set_data([], [])
        else:
            self.plan_artist.set_data(
                (
                    request.throw_origin_xy_m[0],
                    request.receiver_location_xy_m[0],
                ),
                (
                    request.throw_origin_xy_m[1],
                    request.receiver_location_xy_m[1],
                ),
            )
        self.figure.canvas.draw_idle()

    def _on_field_click(self, event) -> None:
        if (
            event.inaxes is not self.field_axes
            or event.button != 1
            or event.xdata is None
            or event.ydata is None
        ):
            return
        x_m = float(event.xdata)
        y_m = float(event.ydata)
        if not 0 <= x_m <= self.field.length_m or not 0 <= y_m <= self.field.width_m:
            return
        if (
            self._throw_origin_xy_m is None
            or self._receiver_location_xy_m is not None
        ):
            self._throw_origin_xy_m = (x_m, y_m)
            self._receiver_location_xy_m = None
            self._set_status("Thrower selected. Click the intended receiver location.")
        else:
            self._receiver_location_xy_m = (x_m, y_m)
            self._set_status("Target set. Evaluate the pass, or select Timing pass to set a catch time.")
        self._clear_analysis()
        self._refresh_positions()

    def set_catch_time(self, catch_time_s: float) -> None:
        if self.planning_mode is not PassPlanningMode.TIMING_PASS:
            raise ValueError("select Timing pass before setting a catch time")
        if (
            isinstance(catch_time_s, bool)
            or not isinstance(catch_time_s, Real)
            or not math.isfinite(catch_time_s)
        ):
            raise ValueError("catch_time_s must be a finite positive number")
        if catch_time_s <= 0:
            raise ValueError("catch_time_s must be a finite positive number")
        self._catch_time_s = float(catch_time_s)
        self._time_input_error = None
        self._clear_analysis()
        self._set_status("Catch time updated. Evaluate the pass to refresh results.")

    def _on_catch_time_submitted(self, text: str) -> None:
        if self.planning_mode is not PassPlanningMode.TIMING_PASS:
            return
        try:
            value = float(text)
            self.set_catch_time(value)
        except ValueError as error:
            self._clear_analysis()
            self._time_input_error = str(error)
            self.analysis_error = str(error)
            self._set_status(f"Invalid catch time: {error}", error=True)

    def set_planning_mode(self, mode: PassPlanningMode) -> None:
        if not isinstance(mode, PassPlanningMode):
            raise ValueError("mode must be a PassPlanningMode")
        if self.mode_widget.value_selected != mode.value:
            self.mode_widget.set_active(tuple(PassPlanningMode).index(mode))
            return
        self.catch_time_widget.stop_typing()
        self.planning_mode = mode
        self._time_input_error = None
        self._clear_analysis()
        timing_pass = mode is PassPlanningMode.TIMING_PASS
        self.catch_time_widget.eventson = False
        self.catch_time_widget.set_val(str(self._catch_time_s))
        self.catch_time_widget.eventson = True
        self.catch_time_widget.set_active(timing_pass)
        self.time_axes.set_visible(timing_pass)
        self.catch_time_artist.set_visible(not timing_pass)
        self._set_status(f"{mode.value} selected. Evaluate the pass to refresh results.")

    def _on_planning_mode_selected(self, label: str) -> None:
        self.set_planning_mode(PassPlanningMode(label))

    def set_throw_type(self, throw_type: ThrowType) -> None:
        if not isinstance(throw_type, ThrowType):
            raise ValueError("throw_type must be a ThrowType")
        self.throw_type = throw_type
        self._clear_analysis()
        self._set_status(f"{throw_type.value.title()} selected. Evaluate the pass to refresh results.")

    def _on_throw_type_selected(self, label: str) -> None:
        self.set_throw_type(ThrowType(label))

    def _on_evaluate_clicked(self, _event) -> None:
        self.evaluate()

    def evaluate(self) -> None:
        """Solve and evaluate the current plan, surfacing failures in the view."""
        request = self.target_request
        self._clear_analysis()
        if request is None:
            self._set_status(
                "Select both a thrower and an intended receiver before evaluating.",
                error=True,
            )
            return
        if (
            self.planning_mode is PassPlanningMode.TIMING_PASS
            and self._time_input_error is not None
        ):
            self._set_status(f"Invalid catch time: {self._time_input_error}", error=True)
            return

        throw_profile = request.throw_profile
        self._set_status("Searching for a verified release. This may take a while...")
        self.figure.canvas.flush_events()
        try:
            if self.planning_mode is PassPlanningMode.TARGET_ONLY:
                result = plan_target_pass(request, self.disc, timing=self.timing)
                release = result.release_result
                planned = result.planned_pass
                diagnostic = result.diagnostic
            else:
                planned = request.at_catch_time(self._catch_time_s)
                release = solve_nominal_release(planned, throw_profile, self.disc)
                diagnostic = release.diagnostic
            self.release_result = release
            if not release.feasible:
                self._set_status(
                    diagnostic,
                    error=True,
                )
                return

            if release.trajectory is None or planned is None:
                raise RuntimeError("verified release is missing its plan or trajectory")
            self._resolved_pass = planned
            self.catch_time_artist.set_text(
                f"Catch time: {planned.nominal_catch_time_s:.3f} s (automatic)"
            )
            positions = np.asarray(
                [sample.position_m for sample in release.trajectory.samples],
                dtype=float,
            )
            self.trajectory_artist.set_data(positions[:, 0], positions[:, 1])
            self.catchability_result = evaluate_flight_catchability(
                planned,
                release,
                throw_profile,
                self.variation_profiles[self.throw_type],
                self.disc,
                np.random.default_rng(self.random_seed),
                trial_count=self.trial_count,
            )
        except (RuntimeError, ValueError) as error:
            self._clear_analysis()
            self.analysis_error = str(error)
            self._set_status(f"Analysis failed: {error}", error=True)
            return

        self._set_status(
            f"Flight-level catchability: "
            f"{self.catchability_result.successful_trial_count}/"
            f"{self.catchability_result.total_trial_count} "
            f"({self.catchability_result.catchability_fraction:.1%}).\n"
            f"Nominal catch time: {planned.nominal_catch_time_s:.3f} s.\n"
            f"Nominal target error: {release.target_error_m:.3f} m.\n"
            f"Not full pass-completion probability.\n"
            f"Aerodynamics: {self.catchability_result.flight_model_provenance}\n"
            f"Throw variation: {self.catchability_result.throw_variation_provenance}",
        )
        self.figure.canvas.draw_idle()

    def render(self) -> Figure:
        """Draw the current editor state and return its figure."""
        self.figure.canvas.draw()
        return self.figure

    def close(self) -> None:
        """Close the editor window and release its Matplotlib figure."""
        import matplotlib.pyplot as plt

        plt.close(self.figure)


def create_scenario_editor(
    field: FieldGeometry = FieldGeometry(),
    disc: DiscProfile | None = None,
    *,
    trial_count: int = 20,
    random_seed: int = 42,
    timing: AutomaticTimingSettings = AutomaticTimingSettings(),
) -> ScenarioEditor:
    """Create an editor without opening its window."""
    return ScenarioEditor(
        field,
        disc,
        trial_count=trial_count,
        random_seed=random_seed,
        timing=timing,
    )


def show_scenario_editor(
    field: FieldGeometry = FieldGeometry(),
    disc: DiscProfile | None = None,
    *,
    trial_count: int = 20,
    random_seed: int = 42,
    timing: AutomaticTimingSettings = AutomaticTimingSettings(),
) -> ScenarioEditor:
    """Open the local scenario editor and return it after the window closes."""
    import matplotlib.pyplot as plt

    editor = create_scenario_editor(
        field,
        disc,
        trial_count=trial_count,
        random_seed=random_seed,
        timing=timing,
    )
    plt.show()
    return editor
