## Code Usage Examples:

### Calling the `FieldRelativeShooterSolver`:

    import edu.wpi.first.math.geometry.*;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;

    // ...

    // 1) Current robot pose in FIELD coordinates (e.g. from pose estimator)
    Pose3d fieldRobotPose = poseEstimator.getEstimatedPosition();

    // 2) Fixed transform from robot origin to launcher exit
    //    (measured once and stored in Constants)
    Transform3d launcherTransformRobot = ShooterConstants.kLauncherTransform;

    // 3) Target pose in FIELD coordinates
    //    (from vision, AprilTag map, or a fixed field location)
    Pose3d fieldTargetPose = targetProvider.getTargetPose();

    // 4) Current robot/platform velocity in FIELD coordinates
    //    (convert from ChassisSpeeds)
    ChassisSpeeds speeds = drivetrain.getChassisSpeeds();

    Translation2d fieldPlatformVelocity =
            new Translation2d(speeds.vxMetersPerSecond,
                              speeds.vyMetersPerSecond);

    // 5) Solve for shot parameters
    FieldRelativeShooterSolver.FieldShotSolution solution =
            FieldRelativeShooterSolver.solve(
                    fieldRobotPose,
                    launcherTransformRobot,
                    fieldTargetPose,
                    fieldPlatformVelocity
            );

    // 6) Use the solution
    double v0 = solution.v0();                    // m/s launch speed
    Rotation2d aimYaw = solution.psiField();      // field-relative azimuth
    double flightTime = solution.time();           // seconds

    // Optional diagnostics
    double apexHeight = solution.zApex();
    double rUp = solution.crossingsAtDz().rUp();
    double rApex = solution.crossingsAtDz().rApex();
    double rDown = solution.crossingsAtDz().rDown();

    // 7) Apply to hardware
    launcher.setTargetExitVelocity(v0);
    turret.setFieldRelativeYaw(aimYaw);


--------

## Notes we’ll care about later


### Where each input usually comes from

| Input                    | Typical Source                        |
|--------------------------|---------------------------------------|
| `fieldRobotPose`         | `SwerveDrivePoseEstimator`            |
| `launcherTransformRobot` | Measured CAD offset (`Constants`)     |
| `fieldTargetPose`        | AprilTag pose or fixed field constant |
| `fieldPlatformVelocity`  | `ChassisSpeeds` from drivetrain       |


### What to do if the solve fails

The solver throws `IllegalArgumentException` for unreachable geometry.
A production pattern looks like this:

    try {
        var solution = FieldRelativeShooterSolver.solve(
                fieldRobotPose,
                launcherTransformRobot,
                fieldTargetPose,
                fieldPlatformVelocity
        );
        shooter.apply(solution);
    } catch (IllegalArgumentException ex) {
        // Target unreachable under constraints:
        // - too close
        // - apex clearance violated
        // - descending constraint violated
        shooter.disableOrFallback();
    }

### Typical control loop placement

This call is:
* cheap (microseconds)
* deterministic
* safe to run every 20 ms

So it’s appropriate in:
* Command.execute()
* a periodic shooter controller
* an auto-aim loop


--------

## Advanced Topics

Below is a practical, WPILib-idiomatic way to do both:

1. Smooth v0 and psi frame-to-frame (handles discontinuities, noise, and "aim jitter")
2. Wire it into a Command-based auto-aim example you can drop into an FRC project.

Everything is written to be "robot-code friendly" (no fancy dependencies, deterministic, tunable).

### Smoothing strategy

`v0` smoothing

Use a standard 1st-order low-pass filter (exponential moving average):

    v0Filt = v0Filt + α*(v0Meas - v0Filt)


Where `α = dt / (τ + dt)` and `τ` is a time constant (e.g. 0.15–0.30 s).

`psi` smoothing (important!)

Angles wrap at ±π. You **must** smooth using angle error:

    err = angleModulus(psiMeas - psiFilt)
    psiFilt = angleModulus(psiFilt + α*err)


Optionally add a **slew-rate limiter** so aim commands don’t jump faster than your turret can physically follow.


### A small reusable "smoothed setpoint" helper

    import edu.wpi.first.math.MathUtil;
    import edu.wpi.first.math.geometry.Rotation2d;

    public final class SmoothedShotSetpoint {
        private double v0Filt;
        private double psiFiltRad;
        private boolean initialized = false;

        // Tuning
        private final double tauSeconds;        // LPF time constant
        private final double maxPsiRateRadPerS; // turret rate limit

        public SmoothedShotSetpoint(double tauSeconds, double maxPsiRateRadPerS) {
            this.tauSeconds = tauSeconds;
            this.maxPsiRateRadPerS = maxPsiRateRadPerS;
        }

        public void reset(double v0, Rotation2d psi) {
            v0Filt = v0;
            psiFiltRad = psi.getRadians();
            initialized = true;
        }

        public boolean isInitialized() {
            return initialized;
        }

        public void update(double v0Meas, Rotation2d psiMeas, double dtSeconds) {
            if (!initialized) {
                reset(v0Meas, psiMeas);
                return;
            }

            // LPF coefficient
            double alpha = dtSeconds / (tauSeconds + dtSeconds);

            // Smooth v0
            v0Filt = v0Filt + alpha * (v0Meas - v0Filt);

            // Smooth psi with wrap-safe error
            double psiMeasRad = psiMeas.getRadians();
            double err = MathUtil.angleModulus(psiMeasRad - psiFiltRad);
            double psiNew = MathUtil.angleModulus(psiFiltRad + alpha * err);

            // Slew-rate limit psi
            double maxStep = maxPsiRateRadPerS * dtSeconds;
            double step = MathUtil.clamp(
                    MathUtil.angleModulus(psiNew - psiFiltRad),
                    -maxStep,
                    +maxStep
            );

            psiFiltRad = MathUtil.angleModulus(psiFiltRad + step);
        }

        public double v0() {
            return v0Filt;
        }

        public Rotation2d psi() {
            return Rotation2d.fromRadians(psiFiltRad);
        }
    }


Suggested starting values
* `tauSeconds`: 0.20 s
* `maxPsiRateRadPerS`: match turret capability, e.g. 3–6 rad/s (≈170–340 deg/s)

### Command-based auto-aim example

Assumptions

* `Drivetrain` subsystem:
  * `Pose3d getPose3d()`
  * `ChassisSpeeds getChassisSpeeds()`

* `Launcher` subsystem:
  * `Transform3d getLauncherTransformRobot()` (or constant)
  * `void setTargetExitVelocity(double v0)`

* `Turret` subsystem:
  * `void setFieldRelativeYaw(Rotation2d yaw)`

* `TargetProvider`:
  * `Pose3d getTargetPoseField()`

And your solver:

    FieldRelativeShooterSolver.solve(...) returning FieldShotSolution

The command:

    import edu.wpi.first.math.geometry.*;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj2.command.Command;

    public class AutoAimCommand extends Command {

        private final Drivetrain drivetrain;
        private final Launcher launcher;
        private final Turret turret;
        private final TargetProvider targetProvider;

        private final SmoothedShotSetpoint smoother;

        private double lastTimestamp = 0.0;

        public AutoAimCommand(
                Drivetrain drivetrain,
                Launcher launcher,
                Turret turret,
                TargetProvider targetProvider
        ) {
            this.drivetrain = drivetrain;
            this.launcher = launcher;
            this.turret = turret;
            this.targetProvider = targetProvider;

            // Tune these:
            this.smoother = new SmoothedShotSetpoint(
                    0.20,   // tau seconds
                    4.0     // max psi rate rad/s
            );

            addRequirements(launcher, turret);
            // Drivetrain typically not required unless you also drive in this command
        }

        @Override
        public void initialize() {
            lastTimestamp = Timer.getFPGATimestamp();
            // Don’t reset smoother here; let it initialize on first solve result
        }

        @Override
        public void execute() {
            double now = Timer.getFPGATimestamp();
            double dt = now - lastTimestamp;
            lastTimestamp = now;

            Pose3d fieldRobotPose = drivetrain.getPose3d();
            Transform3d launcherTransformRobot = launcher.getLauncherTransformRobot();
            Pose3d fieldTargetPose = targetProvider.getTargetPoseField();

            ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
            Translation2d fieldVel = new Translation2d(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond
            );

            try {
                FieldRelativeShooterSolver.FieldShotSolution raw =
                        FieldRelativeShooterSolver.solve(
                                fieldRobotPose,
                                launcherTransformRobot,
                                fieldTargetPose,
                                fieldVel
                        );

                // Smooth setpoints
                smoother.update(raw.v0(), raw.psiField(), dt);

                // Apply smoothed setpoints
                launcher.setTargetExitVelocity(smoother.v0());
                turret.setFieldRelativeYaw(smoother.psi());

                // Optional: diagnostics
                // SmartDashboard.putNumber("Shot/v0_raw", raw.v0());
                // SmartDashboard.putNumber("Shot/v0_filt", smoother.v0());
                // SmartDashboard.putNumber("Shot/psi_raw_deg", raw.psiField().getDegrees());
                // SmartDashboard.putNumber("Shot/psi_filt_deg", smoother.psi().getDegrees());
                // SmartDashboard.putNumber("Shot/apex", raw.zApex());

            } catch (IllegalArgumentException ex) {
                // Unreachable shot under constraints.
                // Best practice: hold last setpoints (don’t jump), or fall back.
                // Here: hold last smoothed output if initialized; otherwise do nothing.
                if (smoother.isInitialized()) {
                    launcher.setTargetExitVelocity(smoother.v0());
                    turret.setFieldRelativeYaw(smoother.psi());
                } else {
                    launcher.stop();
                    turret.hold();
                }

                // Optional: SmartDashboard.putString("Shot/error", ex.getMessage());
            }
        }

        @Override
        public void end(boolean interrupted) {
            // Usually you either hold aim or stop
            launcher.stop();
            turret.hold();
        }

        @Override
        public boolean isFinished() {
            return false; // runs while held
        }
    }

### Wiring it in RobotContainer

Example: run while a driver button is held.

    driverController.rightBumper().whileTrue(
            new AutoAimCommand(drivetrain, launcher, turret, targetProvider)
    );

### Practical tuning notes (what usually matters)

If the turret "hunts" left/right:
* increase `tauSeconds` (more smoothing), or
* lower `maxPsiRateRadPerS`

If it feels sluggish:
* reduce `tauSeconds`, or
* increase `maxPsiRateRadPerS`

If `v0` oscillates:
* often the target pose is noisy → smooth target pose too (separately), or bump `tauSeconds`
