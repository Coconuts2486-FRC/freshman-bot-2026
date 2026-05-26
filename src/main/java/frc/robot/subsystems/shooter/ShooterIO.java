// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends RBSIIO {

  @AutoLog
  public static class ShooterIOInputs {
    public boolean leaderAlive = false;
    public boolean followerAlive = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double positionRadFollower = 0.0;
    public double velocityRadPerSecFollower = 0.0;
    public double velocityMetersPerSecFollower = 0.0;
    public double appliedVoltsFollower = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Run closed loop at the specified velocity.
   *
   * @param velocityRotationsPerSecond Specified velocity in rot / sec
   */
  public default void setVelocity(double velocityRotationsPerSecond) {}

  public default void set(double set) {}

  public default void stop() {}

  /** Set gain constants */
  public default void configureGains(double kP, double kI, double kD, double kS, double kV) {}

  /** Set gain constants */
  public default void configureGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  public default double get() {
    return 0.0;
  }

  public default Angle getPositionRot() {
    return Rotations.of(0.0);
  }

  public default AngularVelocity getVelocityRotPerSec() {
    return RotationsPerSecond.of(0.0);
  }

  public default Voltage getMotorVoltage() {
    return Volts.of(0.0);
  }

  public default double getVelocityRPM() {
    return 0.0;
  }
}
