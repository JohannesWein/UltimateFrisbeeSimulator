# Proposal

## Why

The current aerodynamic calculation treats the spinning disc's body x/y axes as a persistent flight direction. A horizontal disc moving at 18 m/s consequently changes its world-space vertical aerodynamic force from approximately +1.16 N to -1.16 N after half a spin, which can make ordinary passes fail nominal-release search. Correct this structural error before building tactical decisions on flight-level probabilities.

## What Changes

- Compute aerodynamic forces and moments in a flow-aligned frame defined by wind-relative velocity and the disc normal, independent of rotation about that normal.
- Define consistent nose-up/nose-down angle-of-attack signs alongside the corrected flow geometry.
- Preserve quaternion orientation, rigid-body angular momentum, and gyroscopic precession; distinguish spin phase invariance from changing spin rate or direction.
- Define finite, phase-independent behavior for zero airflow and airflow parallel to the disc normal.
- **BREAKING**: Interpret aerodynamic coefficient axes as flow-frame axes, not rotating material x/y axes. Retain public model shapes and body-frame wrench output, but evaluate the existing coefficient surfaces at zero sideslip for the axisymmetric disc. Document changed semantics and resulting trajectory changes for custom profiles.
- Add force, moment, trajectory, and pass-planning regressions, including the approximately 28 m diagonal pass shown by the user.
- Keep coefficient values explicitly provisional. Do not tune coefficients, expand release-attitude search, or increase search budgets merely to make that pass succeed.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `disc-flight-physics`: Require spin-phase-independent aerodynamic loads and phase-equivalent trajectories for axisymmetric discs, while retaining wind-relative flow, spin dynamics, and explicit model provenance.

## Impact

- Primary implementation: `src/ultimate_frisbee_simulator/aerodynamics.py`; integration compatibility with `simulation.py` and the axisymmetric UltraStar inertia profile.
- Regression coverage: aerodynamic unit tests, flight simulation tests, nominal release search, and the scenario editor's resolved-plan and catchability path.
- Documentation: aerodynamic frame conventions, custom coefficient compatibility, and limitations in `README.md`.
- No new dependencies, cloud services, defense behavior, or changes to target-only versus timing-pass contracts.
- The existing completed editor change remains separate and unarchived. This change repairs the physics it consumes rather than reopening its planning scope.
