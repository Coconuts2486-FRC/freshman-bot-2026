package frc.robot.subsystems.extender;

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

  public default void goToPos(double Pos) {}

  public default void setVelocity(double velocity) {}

  public default void stop() {}
}
