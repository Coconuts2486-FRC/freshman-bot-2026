# Fixed-Theta Ballistic Solver — Design Doc

**Purpose**
Compute a physically valid ballistic shot (launch speed `v0` and azimuth `psi`) from a moving platform to a target, using a **fixed elevation angle**, **descending impact**, and a required **apex clearance**. Designed for WPILib / FRC projects with deterministic behavior and unit tests.

---

## 1) Physics Model (Source of Truth)

The solver is derived directly from the original kinematic equations:

Δx = (v0cos(psi)cos(theta) + VRcos(phi)) * t
Δy = (v0sin(psi)cos(theta) + VRsin(phi)) * t
Δz = v0sin(theta)t - 0.5gt^2


Where:

- `v0`    = launch speed
- `theta` = **fixed elevation angle above horizontal**
- `psi`   = **absolute azimuth of the launch velocity**
- `VR`    = platform horizontal speed
- `phi`   = platform velocity direction
- `g`     = gravitational acceleration
- `t`     = time of flight
- `(Δx, Δy, Δz)` = displacement from launcher exit to target

No approximations are made to these equations.

---

## 2) Fixed Design Constraints

These are intentional solver design choices:

- **Fixed elevation**
  - `theta` is constant (e.g. 70°)
  - The solver does *not* search over elevation
- **Gravity-only vertical dynamics**
  - No drag, Magnus, or lift
- **Horizontal platform motion only**
  - Platform velocity has no vertical component
- **Inertial launcher frame**
  - Frame rotation handled explicitly via Pose3d / Transform3d

---

## 3) Trajectory Shape Constraints

The following physical constraints are enforced:

- **Reachability**
  - A real-valued solution must exist
- **Descending impact**
  - Vertical velocity at impact must be negative:

vz(t_hit) = v0sin(theta) - gt_hit < 0


- **Apex clearance**
  - Apex must exceed target height by at least `h_c`:

z_apex = (v0^2 * sin(theta)^2) / (2*g)
z_apex >= Δz + h_c


- **Ascending solutions are rejected**
  - If the endpoint lies on the rising branch, the solve fails

---

## 4) Time-of-Flight (Branch Selection)

The solver **always uses the descending root** of the vertical equation:

t_down(v0) =
(v0sin(theta) + sqrt(v0^2sin(theta)^2 - 2gΔz)) / g


The ascending root is computed only for diagnostics.

---

## 5) Horizontal Consistency Constraint (Core Solve)

At the endpoint, horizontal motion must be consistent with launch speed and platform motion:

| (D / t(v0)) - V | = v0*cos(theta)


Where:

- `D = [Δx, Δy]`
- `V = [VRx, VRy]` (platform velocity in launcher frame)

This equation:

- Couples `v0` to time-of-flight
- Is **non-algebraic** when platform motion is present
- Requires numerical solution when `VR != 0`

---

## 6) Numerical Solution Strategy

Because no closed-form solution exists with platform velocity:

- **Unknown**: `v0` (1D)
- **Method**: Bisection
- **Why bisection**
  - Guaranteed convergence
  - Deterministic runtime
  - No derivatives or tuning
- **Lower bounds**
  - Must reach `Δz`
  - Must satisfy apex clearance
- **Upper bound**
  - Safety cap (e.g. 100 m/s)
- **Monotonicity**
  - Required `v0` increases with range
  - Enforced by unit tests

---

## 7) Coordinate Frames (WPILib Integration)

**Inputs**
- `Pose3d fieldRobotPose`
- `Transform3d launcherTransformRobot`
- `Pose3d fieldTargetPose`
- `Translation2d fieldPlatformVelocity`

**Procedure**
1. Compute launcher pose in field frame:
fieldLauncherPose = fieldRobotPose + launcherTransformRobot

2. Express target in launcher frame
3. Rotate platform velocity into launcher frame
4. Solve physics in launcher frame
5. Convert azimuth back to field frame:
psi_field = psi_launcher + launcherYaw


---

## 8) Diagnostic Quantities & Safety Checks

The solver computes:

- `r_up`    – horizontal distance where trajectory crosses Δz ascending
- `r_apex`  – horizontal distance to apex
- `r_down`  – horizontal distance to descending crossing (endpoint)

Required ordering:

r_up < r_apex < r_down


Violations indicate invalid geometry or a solver bug.

---

## 9) Unit Test Coverage

### Physics-only tests
- Known reference cases (e.g. R = 5 m, Δz = 1.32 m)
- Apex clearance enforcement
- Ascending vs descending rejection
- **Monotonicity sanity**:
R2 > R1 => v0(R2) > v0(R1)


### Field-frame integration tests
- Yaw correctness under robot rotation
- Platform velocity toward target reduces `v0`
- Platform velocity away from target increases `v0`
- Sideways platform motion shifts azimuth

All tests run via:

./gradlew test


---

## 10) Explicit Non-Assumptions

The solver does **not** assume:

- Minimum-energy trajectories
- Symmetric arcs
- Zero platform motion
- Small-angle approximations
- Adjustable elevation

---

## 11) Summary

This solver computes a **fixed-elevation**, **descending-impact**, **apex-cleared** ballistic solution that:

- Exactly satisfies the original physics equations
- Correctly compensates for platform motion
- Is deterministic, testable, and robust
- Is suitable for real-time FRC use and long-term maintenance
