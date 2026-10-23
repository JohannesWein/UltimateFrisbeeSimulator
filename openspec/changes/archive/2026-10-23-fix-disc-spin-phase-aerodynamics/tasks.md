# Tasks

## 1. Aerodynamic Geometry and Coefficient Semantics

- [x] 1.1 Add aerodynamic regression tests reproducing spin-phase-dependent lift, drag, and moments, using horizontal and tilted discs, still air and wind, phases 0/45/90/180/270 degrees, and nonzero custom coefficients in every channel; verify the current evaluator fails the invariance assertions before applying the fix.
- [x] 1.2 Implement flow-aligned lift, drag, side-force, and moment transformations in `aerodynamics.py`, preserving body-frame wrench output and actual body-relative velocity diagnostics; test world force/moment invariance to 1e-9 N/Nm, drag opposition, lift perpendicularity, positive nose-up attack, negative nose-down attack, and zero aerodynamic sideslip with existing coefficient surfaces.
- [x] 1.3 Handle zero and near-normal airflow with the documented axis-only degenerate policy; test exact zero, both normal-flow signs, relative in-plane speeds below and above the 1e-12 threshold, finite outputs, and phase invariance without arbitrary material-axis fallback.
- [x] 1.4 Update evaluator/model docstrings and README frame conventions, angle signs, zero-sideslip surface interpretation, and compatibility notes; verify documentation agrees with tests, public data shapes remain unchanged, and provisional coefficient table values and provenance have not been tuned or relabeled as measured.

## 2. Coupled Flight and Spin Verification

- [x] 2.1 Add phase-equivalent trajectory tests with axisymmetric inertia, equivalent world angular velocity, and common exact samples at 0.1/0.3/0.5 s; verify position/velocity/normal/world-angular-velocity differences stay within 1e-4 m, 1e-4 m/s, 1e-5, and 1e-3 rad/s respectively, using standard integration tolerances and unchanged quaternion dynamics.
- [x] 2.2 Add controlled pitching-torque regressions for opposite spin directions and different spin magnitudes, and revise only invalid old trajectory expectations; verify nonzero disc-normal evolution, distinct gyroscopic responses, repeatability, wind handling, and ground/duration termination with `uv run python -m unittest tests.test_aerodynamics tests.test_flight_simulation -v`.

## 3. Reported Pass and Consumer Compatibility

- [x] 3.1 Add an end-to-end regression for the approximate screenshot backhand from (36.5,17.0) to (63.0,26.5), using the default disc, heights, and unchanged automatic search settings; require a verified exact-time catch within 0.5 m, no earlier ground contact, immutable resolved intent through seeded sampling/catchability, and headless editor rendering, and record attempted times/runtime. If still unresolved, diagnose the remaining limitation and obtain scope approval before extending physics/search; leave this task open rather than claiming success or weakening acceptance.
- [x] 3.2 Verify short-pass and explicit-timing workflows, failure diagnostics, and provisional provenance against the corrected physics; run the README planning/variation examples and affected solver, catchability, variation, and editor tests, updating README examples only when warranted by corrected behavior and never substituting times in timing mode.

## 4. Integration Verification

- [x] 4.1 Run `uv run python -m unittest discover -v`, `uv build`, `git diff --check`, and `openspec.cmd validate fix-disc-spin-phase-aerodynamics --strict`; verify all succeed, no unrelated changes were introduced, and the reported-pass acceptance gate is resolved before marking the change complete.

## Workflow follow-up

- Review the corrected model and its documented limitations before archiving this change.
- Keep the completed tactical-editor change separate; do not archive, commit, or publish either change without a user request.
