// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.computations.FieldRelativeShooterSolver;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends RBSISubsystem {

  // Declare IO
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine sysId;
  private double targetRpm = 0.0;
  // private double shooterOffset = 0.0;

  FieldRelativeShooterSolver.FieldShotSolution solution;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configureGains(kPreal, 0.0, kDreal, kSreal, kVreal, kAreal);
        break;
      case SIM:
      default:
        io.configureGains(kPsim, 0.0, kDsim, kSsim, kVsim, kAsim);
        break;
    }

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runVolts(voltage.in(Volts)),
                null, // <- IMPORTANT for AdvantageKit
                this));
  }

  /** Periodic function -- inherits timing logic from RBSISubsystem */
  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/targetspeed", targetRpm);
    Logger.recordOutput("Shooter/currentSpeed", io.get());
    Logger.recordOutput("Shooter/currentSpeedRPM", io.getVelocityRPM());
    Logger.recordOutput("Shooter/atSpeed", shooterAtSpeed());
    Logger.recordOutput("Shooter/flywheelGoing", Math.abs(io.get()) > 0);
    // Logger.recordOutput("Shooter/ShooterOffset", shooterOffset);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Set a duty cycle percentage
   *
   * @param set Set point
   */
  public void set(double set) {
    // Simple 3-point regression to convert DutyCycle to RPM
    io.set(set);
  }

  /**
   * Run the motor in closed-loop RPM mode
   *
   * @param velocityRPM The motor speed in RPM
   */
  public void runVelocityRPM(double velocityRPM) {

    double speed = velocityRPM / 60.;
    targetRpm = velocityRPM;

    // Set the motor velocity in rotations / second
    io.setVelocity(speed);
  }

  public void runTargetVelocity(
      Pose3d robotPose,
      Transform3d launcherTransform,
      Pose3d targetPose,
      Translation2d platformVelocity) {

    solution =
        FieldRelativeShooterSolver.solve(
            robotPose, launcherTransform, targetPose, platformVelocity);

    runVelocity(0);

    System.out.println(solution.v0());
  }

  /**
   * Run closed loop at the specified velocity
   *
   * @param velocity Target wheel speed velocity in meters / second
   */
  public void runVelocity(double velocity) {

    // This is radians per second
    double speed =
        (velocity / ShooterConstants.kFlywheelCircumfrence) * ShooterConstants.kShooterGearRatio;

    // rad/s -> RPM
    targetRpm = speed / (2 * Math.PI) * 60.0;

    // Set the rotations/second to the motor
    io.setVelocity(speed / (2 * Math.PI));

    // Log Shooter setpoint
  }

  /** Stops the Shooter. */
  public void stop() {
    io.stop();
  }

  // public void incrementOffset(double change) {
  //   shooterOffset = shooterOffset + change;
  // }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput(key = "Mechanism/Shooter")
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean shooterAtSpeed() {
    if (targetRpm == 0.0) return false;
    double currentSpeed = Math.abs(io.getVelocityRPM());
    // inputs.velocityRadPerSec / 425 * -1;
    return currentSpeed >= Math.abs(targetRpm) * 0.9;
  }

  public boolean shooterAlmostAtSpeed() {
    if (targetRpm == 0.0) return false;
    double currentSpeed = Math.abs(io.getVelocityRPM());
    // inputs.velocityRadPerSec / 425 * -1;
    return currentSpeed >= Math.abs(targetRpm) * 0.75;
  }

  public boolean leaderAlive() {
    return inputs.leaderAlive;
  }

  public boolean followerAlive() {
    return inputs.followerAlive;
  }

  // public double shooterOffset() {
  //   return shooterOffset;
  // }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
