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

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExtenderIOInputs inputs) {}

  public default void extendBlocker() {}

  public default void retractBlocker() {}
}
