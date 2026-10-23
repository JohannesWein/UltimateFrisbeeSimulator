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
body axes are x forward, y spanwise, and z along its normal. Attitude is a unit
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
