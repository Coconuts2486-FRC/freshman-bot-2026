package frc.robot.subsystems.extender;

import frc.robot.subsystems.flywheel_example.FlywheelIO.FlywheelIOInputs;
import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO extends RBSIIO {
  @AutoLog
  public static class ExtenderIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean extenderConected = false;
  }

  public default void updateInputs(ExtenderIOInputs inputs) {}

  public default double getPos() {
    return 0.0;
  }

  public default void setPivotVelocity(double velocityInput) {}

  public default void stop() {}

  public default void configPID(double kP, double kI, double kD) {}

  public default double downPos() {
    return 0.0;
  }

  public default void goUntilPosition(double position) {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Run closed loop at the specified velocity using a profiled/smoothed velocity request. */
  public default void setVelocityProfiled(double velocityRadPerSec) {
    setVelocity(velocityRadPerSec);
  }

  /** Set gain constants */
  public default void configureGains(double kP, double kI, double kD, double kS, double kV) {}

  /** Set gain constants */
  public default void configureGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}
}
