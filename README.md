# Ultimate Frisbee Simulator

A local, incrementally built simulation core for exploring Ultimate Frisbee
field geometry and disc flight.

## Field geometry

`FieldGeometry` uses metres and a two-dimensional coordinate plane. The origin
`(0, 0)` is the corner at the start of end zone A. The x-axis follows the
field's length; the y-axis follows its width. The default field is 100 m long
and 37 m wide, with 18 m end zones and a 64 m central playing area.

Positions on the outer field boundary are included. The two goal-line
boundaries are classified as part of their corresponding end zones.

```python
from ultimate_frisbee_simulator import FieldGeometry, FieldRegion

field = FieldGeometry()
assert field.playing_area_length_m == 64.0
assert field.classify_position(10.0, 10.0) is FieldRegion.END_ZONE_A
assert field.classify_position(50.0, 10.0) is FieldRegion.PLAYING_AREA
assert field.classify_position(90.0, 10.0) is FieldRegion.END_ZONE_B
```

Custom dimensions can be supplied in metres:

```python
custom_field = FieldGeometry(
    length_m=120.0,
    width_m=50.0,
    end_zone_depth_m=20.0,
)
assert custom_field.playing_area_length_m == 80.0
```

Non-finite or non-positive dimensions, or dimensions that leave no central
playing area, raise `ValueError`. Non-finite position coordinates also raise
`ValueError`; finite points outside the field return `FieldRegion.OUTSIDE`.

## Disc-flight physics

The flight model uses SI units: positions in metres, time in seconds,
velocities in metres per second, angular velocities in radians per second,
and forces and moments in newtons and newton-metres. World z points up. A disc's
body axes x/y rotate with the material; body z is its normal. Attitude is a unit
Hamilton quaternion in `[w, x, y, z]` order rotating body vectors into world
coordinates; angular velocity is expressed in the body frame.

`discraft_ultra_star()` supplies the named 175 g, 27.3 cm reference profile.
Mass and diameter are attributed to the [manufacturer's Ultra-Star
specification](https://discraft.com/ultimate.html); the reference area is
derived from a circular planform. Principal inertias use an explicitly
identified uniform-thin-disc approximation, not a measurement of a rimmed
Ultra-Star. The default aerodynamic coefficient surfaces are illustrative
provisional assumptions, not calibrated Ultra-Star data. See
[`doc/paper/README.md`](./doc/paper/README.md) for what the Koyanagi et al.
wind-tunnel study measured, its test conditions, and why those results are not
treated as Ultra-Star-specific.

Each disc profile carries an `AerodynamicCoefficientModel`. Its six
`CoefficientSurface`s describe lift, drag, side force, and roll, pitch, and yaw
moment coefficients as functions of angle of attack and sideslip. Surface
values are bilinearly interpolated; values outside the configured angle axes
are clamped to the nearest endpoint. Supply measured curves and their source
as a replacement model when available. The default model's provenance is
available as `disc.aerodynamic_coefficients.provenance`.

Aerodynamic loads use a flow-aligned frame: x follows the wind-relative velocity
projected into the disc plane, y is normal cross x, and z is the disc normal.
Lift follows the normal projected perpendicular to relative velocity; drag
opposes that velocity. Roll/pitch/yaw coefficients act about these flow axes.
Returned forces and moments are transformed back into the body frame for
rigid-body integration. Material spin phase therefore cannot reverse lift or
change world-space loads, while spin rate and direction still affect precession.

Angle of attack is `atan2(-v_normal, in_plane_speed)`, positive for a nose-up
disc relative to its trajectory. Axisymmetric flight evaluation samples every
coefficient surface at zero sideslip and reports zero sideslip; its relative
velocity diagnostic remains the actual body-frame velocity. **Compatibility:**
this replaces the old rotating body-axis convention and angle sign. Custom
nonzero-sideslip surface columns are no longer sampled by flight evaluation,
although direct two-dimensional interpolation remains available. Existing
trajectories can change; coefficient values remain unchanged and provisional.

Zero wind-relative speed gives zero loads. At normal incidence (in-plane speed
at most `1e-12` times total speed), only axial drag and axial yaw moment are
applied; transverse lift, side force, roll, and pitch are zero. This finite,
phase-independent singular-flow policy is not a calibrated high-angle model.

The solver uses SciPy `solve_ivp` with RK45. Default `SolverSettings` are
relative tolerance `1e-8`, absolute tolerance `1e-10`, maximum step `0.01 s`,
and maximum duration `10 s`. `FlightEnvironment` defaults to zero wind,
`1.225 kg/m^3` air density, and `9.80665 m/s^2` gravity. The simulation
integrates translation and rigid-body rotation together, including
gyroscopic coupling from the Euler rigid-body equations. It stops at descending
ground contact or the maximum duration and reports which occurred. Identical
inputs and settings produce identical samples with the same NumPy/SciPy
versions; solver versions and tolerances should be kept fixed for exact
reproducibility across machines.

```python
from ultimate_frisbee_simulator import (
    FieldGeometry,
    FlightInitialState,
    FlightTerminationReason,
    simulate_flight,
    discraft_ultra_star,
)

field = FieldGeometry()
trajectory = simulate_flight(
    disc=discraft_ultra_star(),
    initial_state=FlightInitialState(
        position_m=(32.0, 18.5, 1.8),
        velocity_m_s=(12.0, 0.0, 3.0),
        angular_velocity_body_rad_s=(0.0, 0.0, 50.0),
    ),
)
landing = trajectory.samples[-1]
landing_region = field.classify_position(
    landing.position_m[0],
    landing.position_m[1],
)
assert trajectory.termination_reason in (
    FlightTerminationReason.GROUND_CONTACT,
    FlightTerminationReason.MAXIMUM_DURATION,
)
```

Each `TrajectorySample` contains elapsed time, position, velocity, quaternion
attitude, and body-frame angular velocity. Invalid model and solver parameters
raise `ValueError`; integration failures raise `RuntimeError`.

## Tactical pass planning

### Pass to target (default)

`TargetPassRequest` specifies the origin, receiver target, and throw profile,
without requiring a catch time. `plan_target_pass` tries a deterministic,
ascending grid of catch times using the fixed-time release solver. On success,
it returns an immutable `PlannedPass` with the derived time and a verified
nominal release:

```python
from ultimate_frisbee_simulator import (
    TargetPassRequest,
    ThrowType,
    default_throw_profiles,
    discraft_ultra_star,
    plan_target_pass,
)

profile = default_throw_profiles()[ThrowType.BACKHAND]
request = TargetPassRequest(
    throw_origin_xy_m=(20.0, 18.5),
    receiver_location_xy_m=(26.0, 18.5),
    throw_profile=profile,
)
automatic = plan_target_pass(request, discraft_ultra_star())
assert automatic.feasible, automatic.diagnostic
planned_pass = automatic.planned_pass
release = automatic.release_result
print(planned_pass.nominal_catch_time_s, release.target_error_m)
```

`AutomaticTimingSettings` defaults to 10 evenly spaced candidates from
0.3 to 3.0 seconds (inclusive), with up to 20 optimizer evaluations per candidate.
Finite-difference Jacobian calculations require additional flight integrations;
the optimizer evaluation budget is not an integration count or wall-clock limit.
Search stops at the earliest **verified time among the examined candidates**,
not a globally optimal or physically fastest catch time. Bounds, candidate
count, and per-candidate budget can be passed via `timing=AutomaticTimingSettings(...)`.
The maximum time must fit within `SolverSettings.maximum_duration_s`.

A failed search leaves `automatic.planned_pass` as `None`; inspect
`automatic.diagnostic` and `automatic.attempted_catch_times_s`. It does not prove
physical impossibility. In particular, ground-before-catch describes the last
failed release candidate, not every possible throw. The search varies speed,
aim, and elevation, but keeps the profile's disc attitude and spin fixed.
With the provisional aerodynamic model, some ordinary-looking passes may not
resolve. Searches can take tens of seconds or several minutes; the local editor
evaluates synchronously and does not provide progress or cancellation controls.

### Timing pass (explicit catch time)

`PlannedPass` stores the throw origin, intended receiver location, and nominal
catch time independently of the release that executes it. Field positions are
2D metres; release and catch heights are separate 3D flight inputs. The bounded
`solve_nominal_release` API searches release speed, horizontal aim offset, and
elevation within the selected `ThrowProfile`. Its exact-time trajectory sample
must be within the plan's target tolerance, and the disc must remain airborne
until that time. Otherwise, the result has `feasible == False` and an explicit
`ReleaseFailure`.

The initial backhand and forehand profiles use provisional generic constraints:
5-25 m/s release speed, -10 to 55 degrees elevation, up to 45 degrees aim
offset, and opposite 50 rad/s spin conventions. These are not calibrated
athlete or manufacturer measurements. The flight model's aerodynamic
coefficients are also provisional; a feasible result is a model-based nominal
trajectory, not a guarantee of a real-world pass.

```python
from ultimate_frisbee_simulator import (
    PlannedPass,
    ThrowType,
    default_throw_profiles,
    discraft_ultra_star,
    solve_nominal_release,
)

planned_pass = PlannedPass(
    throw_origin_xy_m=(20.0, 18.5),
    receiver_location_xy_m=(26.1845, 18.3099),
    nominal_catch_time_s=0.7,
    target_tolerance_m=0.5,
)
profile = default_throw_profiles()[ThrowType.BACKHAND]
release = solve_nominal_release(
    planned_pass,
    profile,
    discraft_ultra_star(),
)
assert release.feasible, release.failure
print(release.launch_state.velocity_m_s)
```

The intended target and catch time remain unchanged by solving. A feasible
result includes the nominal launch state and simulated trajectory. A timing
pass never substitutes an automatically selected time if the requested time
cannot be resolved. The solver
reports ground contact before the catch time, targets outside release
constraints, optimization failure, and catch times beyond the simulation
duration distinctly.

### Throw variation and flight-level catchability

`default_throw_variation_profiles()` provides separate provisional backhand and
forehand spreads. Each profile defines a zero-distance standard deviation and
a linear increase per metre for release speed, horizontal direction,
elevation, and spin. All are configurable; they are illustrative starting
assumptions, not measured population or player data. Samples use independent
zero-mean truncated normal deviations, constrained by the selected
`ThrowProfile`'s speed, aim, and elevation limits. Spin variation preserves the
profile's spin direction.

The estimate checks each simulated execution against the plan's 3D catch
radius during the nominal catch time plus/minus its time tolerance. Exact-time
samples across that window are spaced no farther apart than `time_step_s`
(default `0.02 s`). A fixed NumPy generator state reproduces both individual
executions and trial outcomes:

The following example uses `planned_pass`, `release`, and `profile` from either
successful planning workflow above. Unresolved plans must not be passed to
sampling or evaluation; both require a feasible nominal release. Neither
operation changes the planned target or resolved catch time.

```python
import numpy as np
from ultimate_frisbee_simulator import (
    default_throw_variation_profiles,
    evaluate_flight_catchability,
    sample_throw_execution,
)

variation = default_throw_variation_profiles()[ThrowType.BACKHAND]
generator = np.random.default_rng(42)
one_execution = sample_throw_execution(
    planned_pass,
    release,
    profile,
    variation,
    generator,
)
estimate = evaluate_flight_catchability(
    planned_pass,
    release,
    profile,
    variation,
    discraft_ultra_star(),
    np.random.default_rng(42),
    trial_count=20,
)
print(estimate.successful_trial_count, estimate.total_trial_count)
print(estimate.catchability_fraction, estimate.interpretation)
print(estimate.flight_model_provenance)
print(estimate.throw_variation_provenance)
```

The reported proportion is **flight-level catchability**, not full pass
completion probability: no receiver movement or defensive interception is
modeled. Aerodynamic-model provenance is returned separately from throw
variation provenance. A flight integration error raises `ThrowEvaluationError`
with its trial number; it is never silently counted as a successful throw.

### Launch the local scenario editor

After installing the project dependencies, start the interactive field view:

```console
uv run python examples/scenario_editor.py
```

Click once on the field to place the thrower and again to set the intended
receiver location. Select backhand or forehand and choose **Evaluate pass**.
The default **Pass to target** mode determines and displays the catch time;
there is no editable time field in this mode. For deliberate timing constraints,
select **Timing pass**, enter a positive catch time and press Enter, then evaluate.
The editor displays the planned pass, the verified nominal trajectory, or a red
search-failure diagnostic, and reports flight-level catchability with separate
aerodynamic and throw-variation provenance. Changing positions, throw profile,
catch time, or planning mode clears the old trajectory and estimate; automatic
mode also clears its derived time until reevaluation. A third field click starts
a new scenario with a new thrower. Invalid timing text blocks evaluation instead
of reusing a previous valid time. The local
editor runs without a cloud service; scenarios are currently held in memory
and are not saved.

## 3D animation

The Matplotlib view displays the field and end zones in 3D, animates the
simulated disc orientation and its recent flight trail, and interpolates
between solver samples for smooth playback. To run the included example:

```console
uv run python examples/animate_flight.py
```

To save a GIF instead of opening a window:

```console
uv run python examples/animate_flight.py --save flight.gif
```

Or create a view from a trajectory in Python:

```python
import matplotlib.pyplot as plt
from ultimate_frisbee_simulator import animate_flight

animation = animate_flight(trajectory, disc)
plt.show()
```

`animate_flight` returns a Matplotlib `FuncAnimation`; keep a reference to it
until the window is displayed. Pass `trail_duration_s` to limit the visible
trail or `frames_per_second` to adjust playback smoothness and speed. In a
headless environment, save the returned animation with Matplotlib's
`PillowWriter` instead of calling `plt.show()`.

## Tactical roadmap

The tactical simulator will grow in separate, incremental changes. The planned
sequence is:

1. Add a top-down scenario editor for the field, thrower, intended receiver
   location, nominal catch time, and configurable throw variation.
2. Add static defender placement and visualize potential passing lanes.
3. Evaluate time-dependent interception reach using defender reaction time and
   running speed.
4. Add force direction and rule-based marking behavior.
5. Model thrower fakes as timed actions that can change the marker's response,
   while preserving pivot and movement constraints.
6. Add autonomous defenders that make decisions from the scenario and react to
   offensive movement and fakes.

Each stage is intended to be planned and implemented as a separate OpenSpec
change. Early pass evaluations describe flight-level catchability only;
defender interception and full pass-completion estimates require the later
defensive behavior stages. Autonomous agents and reinforcement learning remain
later extensions after the rule-based simulation is established.

## Development

Install the project and its dependencies, then run the tests:

```console
uv sync
uv run python -m unittest discover -v
```
