package frc.robot.subsystems.extender;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface extenderIO extends RBSIIO {
  @AutoLog
  public static class extenderIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(extenderIOInputs inputs) {}

  public default double extenderPos() {
    return 0.0;
  }

  public default void goToPos(double Pos) {}

  public default void setVelocity(double velocity) {}

  public default void stop() {}
}
