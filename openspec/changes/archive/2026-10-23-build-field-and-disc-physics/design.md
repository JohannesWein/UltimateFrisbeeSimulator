# Design

## Context

See proposal.md for motivation. The repository has no application code, dependency manifest, or test suite. Python-specific ignore rules suggest Python, but do not prove an existing runtime choice. This change will introduce the Python core and its NumPy/SciPy dependencies. The requirements are in specs/field-geometry/spec.md and specs/disc-flight-physics/spec.md.

## Goals / Non-Goals

**Goals:**
- Keep field geometry and flight physics independently usable and testable.
- Model coupled translational and rotational disc motion, including aerodynamic moments and gyroscopic precession.
- Use explicit SI units and produce reproducible trajectories for identical inputs and numerical settings.

**Non-Goals:**
- Real-time multiplayer rendering or cloud services.
- Computational fluid dynamics, mesh generation, or disc deformation.
- Claiming calibrated real-world accuracy before coefficient data is selected and validated.

## Decisions

1. **Use NumPy and SciPy for numerical modeling.** Declare them as project dependencies. NumPy supports vector and rotation-state calculations; SciPy supplies a maintained adaptive integrator and event handling. This avoids implementing a custom integrator for coupled differential equations.
2. **Model the disc as a coupled rigid body.** Integrate position, linear velocity, attitude, and angular velocity. Represent attitude with a normalized quaternion; calculate translational acceleration from gravity and aerodynamic lift and drag, and angular acceleration from aerodynamic moments and the rigid-body Euler equations using the disc's principal moments of inertia. This captures spin and gyroscopic precession without CFD.
3. **Use body-relative aerodynamic coefficients.** Calculate relative airflow from disc velocity minus wind, transform it into the disc frame, and derive angle of attack and sideslip from that airflow and the current attitude. Parameterize lift, drag, and roll, pitch, and yaw moment coefficients against those angles so empirical coefficient curves can be configured and calibrated rather than embedded in the integrator.
4. **Use SciPy's adaptive `solve_ivp` integration with RK45.** Configure and document absolute and relative tolerances and a maximum integration step so output is reproducible for identical inputs and settings. Use a descending ground-contact event to stop the flight and identify its termination reason; also enforce the configured maximum duration. Reject invalid input and invalid numerical settings explicitly.
5. **Keep the field model independent with a fixed SI coordinate convention.** Field x runs along its length, y along its width, and (0, 0) is one field corner. The first and last end-zone boundaries classify as end-zone positions. In flight, z points up; convert field coordinates and flight positions only at a clearly defined integration boundary if needed.
6. **Use Python's standard `unittest` runner.** It avoids another test dependency while testing field boundaries, force and moment directions, spin/precession response, wind-relative airflow, validation, reproducibility, and flight termination.

## Literature

- Koyanagi, R., Seo, K., Ohta, K., & Ohgi, Y. (2012). *A computer simulation of the flying disc based on the wind tunnel test data*. Procedia Engineering, 34, 80–85. https://doi.org/10.1016/j.proeng.2012.04.015. A copy is available at `doc/paper/A_computer_simulation_of_the_flying_disc_based_on_.pdf`. The study reports wind-tunnel measurements of three force and three moment components and compares simulated flight with motion-capture kinematics. Its tested disc model must be confirmed from the paper before treating its data as UltraStar-specific.
- Potts, J. R., & Crowther, W. J. (2002). *Frisbee(TM) Aerodynamics*. AIAA 2002-3150. https://doi.org/10.2514/6.2002-3150.
- Hummel, S., & Hubbard, M. (2002). *Identification of Frisbee Aerodynamic Coefficients using Flight Data*. 4th International Conference on the Engineering of Sport. This work is cited in Koyanagi et al.; use its flight-data coefficient identification as supporting literature, not as evidence of wind-tunnel measurements.
- Weizman, Y., Tan, A. M., & Fuss, F. K. (2020). *Measurement of Flight Dynamics of a Frisbee Using a Triaxial MEMS Gyroscope*. https://doi.org/10.3390/proceedings2020049066. The reported disc's equivalence to a Discraft UltraStar must not be assumed.

## Risks / Trade-offs

- **Unselected empirical coefficient data limits real-world accuracy.** Keep curves configurable, label defaults as provisional, and do not claim a named model's fidelity until its source and parameters are documented and calibration tests exist.
- **Adaptive integration can be sensitive to tolerances and dependency versions.** Make numerical settings explicit, test repeatability with fixed settings, and record NumPy/SciPy dependency constraints.
- **Coupled rotation and translation increase model complexity.** Keep aerodynamic coefficient evaluation and rigid-body integration as separate testable components.
- **A simplified rigid-body model omits deformation and detailed fluid effects.** Document those limits and calibrate against measured flight data before using the simulator for quantitative coaching conclusions.
- **Published disc datasets may not describe an UltraStar.** Store parameter provenance and distinguish directly applicable UltraStar data from comparative values and assumptions.

## Migration Plan

There is no existing application or persisted data to migrate. Add the Python project metadata, domain modules, dependencies, and tests in place; rollback consists of removing those new files and reverting the declared dependencies.

## Open Questions

- Does Koyanagi et al. identify the tested disc model and provide enough wind-tunnel conditions and coefficient data to use it for the UltraStar profile, or is UltraStar-specific measurement/calibration needed?
