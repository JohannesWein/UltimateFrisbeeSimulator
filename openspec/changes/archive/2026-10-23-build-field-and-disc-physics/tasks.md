# Tasks

## 1. Project Foundation and Field Geometry

- [x] 1.1 Add Python project metadata and declare NumPy and SciPy dependencies; verify the project installs locally and reports the declared dependencies.
- [x] 1.2 Implement the configurable SI field model with regulation defaults, coordinate regions, and dimension validation; verify unit tests cover defaults, custom dimensions, goal-line boundaries, and invalid dimensions.
- [x] 1.3 Document the field coordinate convention and public geometry API; verify the documented examples and run the field geometry tests.

## 2. Coupled Disc Flight Physics

- [x] 2.1 Define the Discraft UltraStar as a named reference-disc profile and validate its 6-DOF launch state, quaternion attitude, angular velocity, physical parameters, principal moments of inertia, environment, and solver settings; verify tests reject non-finite values and invalid physical or numerical parameters.
- [x] 2.2 Review the locally provided Koyanagi et al. paper and record the tested disc model, wind-tunnel conditions, measured force/moment data, and limits of transfer to an UltraStar; verify every adopted value has a traceable source and unsupported equivalence is marked as an assumption.
- [x] 2.3 Implement configurable angle-of-attack and sideslip coefficient curves for lift, drag, and aerodynamic moments using wind-relative airflow; verify unit tests cover force and moment directions and relative-wind calculations.
- [x] 2.4 Integrate the coupled rigid-body equations with SciPy `solve_ivp` and RK45, including gyroscopic precession and ground-contact and maximum-duration termination; verify tests cover curved flight with spin, deterministic repeatability, orientation samples, and both termination conditions.
- [x] 2.5 Document SI units, UltraStar parameter sources, coefficient configuration and provisional-data limits, solver tolerances, rigid-body assumptions, and trajectory results; verify documented examples and all flight-physics tests.

## 3. Integration Verification

- [x] 3.1 Add a smoke test that imports both capabilities and evaluates a flight trajectory relative to a field; verify the full suite passes with `python -m unittest discover -v`.
