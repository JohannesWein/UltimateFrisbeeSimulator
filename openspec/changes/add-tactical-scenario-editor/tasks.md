# Tasks

## 1. Planned Pass and Nominal Release

- [ ] 1.1 Add validated planned-pass and throw-profile models that keep target location and catch time separate from execution; verify unit tests cover valid values and invalid coordinates/times.
- [ ] 1.2 Implement bounded nominal-release solving against the target at the requested catch time, including exact-time trajectory evaluation and explicit infeasible results; verify tests cover reachable, out-of-tolerance, and ground-before-catch cases.
- [ ] 1.3 Document the nominal pass and release-solving API, constraints, and limitations in the README; verify the documented example finds a feasible throw.

## 2. Throw Variation and Evaluation

- [ ] 2.1 Implement configurable throw-type and distance variation for release speed, direction, elevation, and spin; verify tests cover bounded sampling, profile validation, and repeatability with a fixed random generator.
- [ ] 2.2 Implement seeded Monte Carlo evaluation against a configurable spatial/time catch window and report successful-trial count, trial count, and flight-model provenance; verify identical seeds reproduce outcomes and invalid trials are reported explicitly.
- [ ] 2.3 Document variation parameters, provisional calibration status, success criteria, and the distinction between flight-level catchability and full pass completion; verify the documented evaluation example runs.

## 3. Top-Down Scenario View

- [ ] 3.1 Add a Matplotlib field editor for setting throw origin, target, nominal catch time, and throw profile; verify mouse/widget interactions update the scenario in field coordinates.
- [ ] 3.2 Display the planned pass, feasible nominal trajectory, and flight-level catchability result; verify a headless rendering test exercises the view and invalid plans are visibly identified.
- [ ] 3.3 Document how to launch and use the scenario editor locally; verify the documented command starts without requiring a cloud service.

## 4. Integration Verification

- [ ] 4.1 Add an end-to-end test that creates a planned pass, solves its nominal release, evaluates seeded execution variations, and renders the field view; verify the full test suite passes with `uv run python -m unittest discover -v`.
