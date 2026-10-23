# Spec Delta

## ADDED Requirements

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
