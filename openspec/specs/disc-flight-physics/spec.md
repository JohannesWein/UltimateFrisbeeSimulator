# disc-flight-physics Specification

## Purpose

Provides reproducible, physically based disc trajectories including aerodynamic forces, rotational dynamics, and gyroscopic precession from explicit launch and environmental conditions.

## Requirements

### Requirement: Simulate a three-dimensional disc trajectory with rotation and precession
The system SHALL calculate a disc trajectory in metres and seconds from its launch position, linear velocity, rotational velocity, attitude, physical parameters, and environmental conditions.

#### Scenario: Simulate a curved throw with spin
- **WHEN** a throw is simulated with valid launch velocity, spin rate, and orientation
- **THEN** the result contains ordered samples with elapsed time, three-dimensional position, linear velocity, and orientation
- **AND** the trajectory accounts for gravity, aerodynamic drag, lift, and aerodynamic moments that produce gyroscopic precession and curved flight

#### Scenario: Apply environmental wind
- **WHEN** a throw is simulated with a non-zero wind vector
- **THEN** aerodynamic forces and relative airflow are calculated from disc motion relative to the moving airmass

### Requirement: Provide a traceable Discraft UltraStar reference profile
The system SHALL support the Discraft UltraStar as a named reference disc and SHALL keep its physical and aerodynamic parameters explicit, configurable, and traceable to documented sources or identified assumptions.

#### Scenario: Select the UltraStar reference disc
- **WHEN** a simulation selects the Discraft UltraStar reference profile
- **THEN** the profile identifies the disc model and supplies its configured physical and aerodynamic parameters
- **AND** each parameter records its source or is explicitly marked as an assumption requiring measurement or calibration

#### Scenario: Use data from a different disc
- **WHEN** published or measured aerodynamic data is considered for the UltraStar profile and the tested disc or conditions are not established as equivalent
- **THEN** the data is identified as comparative or provisional rather than represented as UltraStar-specific

### Requirement: Produce deterministic and bounded trajectories
The system SHALL produce the same trajectory for identical inputs and SHALL end a simulation at ground contact or its configured maximum duration.

#### Scenario: Repeat a simulation
- **WHEN** the same throw inputs and numerical settings are simulated more than once
- **THEN** each run produces identical trajectory samples

#### Scenario: End a flight
- **WHEN** a simulated disc reaches ground level or the configured maximum duration
- **THEN** the trajectory stops and reports which condition ended the simulation

### Requirement: Validate simulation inputs
The system SHALL report invalid launch, physical, environmental, or numerical parameters as explicit validation errors rather than returning a successful trajectory.

#### Scenario: Reject invalid simulation parameters
- **WHEN** any simulation parameter is non-finite, mass, reference area, air density, or a principal moment of inertia is non-positive, or an integration tolerance, maximum integration step, or maximum duration is non-positive
- **THEN** simulation fails with an explicit validation error

### Requirement: Aerodynamic loads are independent of disc spin phase
For an axisymmetric disc, the system SHALL compute world-space aerodynamic force and moment independently of material rotation about the disc normal when normal direction and wind-relative velocity are unchanged. Coefficient and output-frame conventions SHALL be documented.

#### Scenario: Rotate a horizontal disc through a full spin
- **WHEN** a horizontal reference disc travels at 18 m/s in still air and only its rotation about its normal changes
- **THEN** world-space force and moment remain equal within absolute tolerance 1e-9 N and 1e-9 Nm respectively
- **AND** lift does not reverse and drag does not oscillate with spin phase

#### Scenario: Rotate a tilted disc in wind
- **WHEN** a tilted reference disc with nonzero wind is evaluated at multiple material rotations about the same normal
- **THEN** world-space force and moment remain equal within absolute tolerance 1e-9 N and 1e-9 Nm respectively
- **AND** the result uses wind-relative velocity rather than a material forward direction

#### Scenario: Preserve public output-frame meaning
- **WHEN** an aerodynamic result is consumed by the flight integrator
- **THEN** its force and moment are expressed in the disc body frame
- **AND** transforming those vectors to world space preserves spin-phase invariance
- **AND** aerodynamic coefficient axes and the axisymmetric zero-sideslip convention are documented, including compatibility effects on existing custom surfaces

### Requirement: Aerodynamic loads have consistent flow geometry
The system SHALL calculate drag opposing relative motion and lift perpendicular to relative motion using the disc normal. It SHALL define an angle-of-attack sign convention and finite, spin-phase-independent behavior for zero airflow and airflow parallel to the normal.

#### Scenario: Distinguish disc inclination from flight elevation
- **WHEN** the disc normal is tilted nose-up relative to an otherwise horizontal relative velocity
- **THEN** the documented angle of attack is positive
- **AND** nose-down inclination produces a negative angle of attack
- **AND** rotating material axes about the normal does not change either angle

#### Scenario: Lift and drag directions
- **WHEN** a nondegenerate relative velocity is evaluated with positive lift and drag coefficients and zero side-force coefficient
- **THEN** lift is perpendicular to relative velocity and points toward the component of the disc normal perpendicular to that velocity
- **AND** drag is antiparallel to relative velocity

#### Scenario: Disc matches the wind
- **WHEN** disc velocity equals wind velocity
- **THEN** aerodynamic force and moment are zero and every returned diagnostic is finite

#### Scenario: Airflow is parallel or nearly parallel to the normal
- **WHEN** the airflow has no resolvable in-plane component
- **THEN** no material-axis-dependent transverse direction is introduced
- **AND** force and moment are finite and independent of spin phase
- **AND** the documented degenerate-flow policy is applied

### Requirement: Phase-equivalent flight preserves coupled spin dynamics
For axisymmetric inertia, launches differing only by material spin phase SHALL have equivalent world-space position, velocity, disc normal, and angular velocity within numerical tolerance. The system SHALL retain aerodynamic moments, angular momentum, and gyroscopic response rather than freezing orientation.

#### Scenario: Compare trajectories from different initial spin phases
- **WHEN** identical launches with axisymmetric inertia and equal physical angular velocity are simulated with different material spin phases
- **THEN** their position, velocity, disc normal, and world angular velocity agree at common requested times within the documented regression tolerances
- **AND** equivalent orientation is compared through disc normals rather than identical quaternion components

#### Scenario: Spin still affects response to torque
- **WHEN** nonzero aerodynamic torque is applied to otherwise identical spinning discs at different spin rates or opposite spin directions
- **THEN** the coupled simulation retains the corresponding distinct angular-momentum and precession response
- **AND** spin phase invariance does not force spin rate or spin direction invariance
