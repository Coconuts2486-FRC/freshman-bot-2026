package frc.robot.subsystems.extender;

import static frc.robot.Constants.ExtenderConstants.*;

import frc.robot.Constants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Extender extends RBSISubsystem {
  private final ExtenderIO io;
  private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

  public Extender(ExtenderIO io) {
    this.io = io;

    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configureGains(kRealP, 0.0, kRealD, kRealS, kRealV, kRealA);
        break;
      case SIM:
      default:
        io.configureGains(kSimP, 0.0, kSimD, kSimS, kSimV, kSimA);
        break;
    }
  }

  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Extender", inputs);
  }

  public void configPID(double kP, double kI, double kD) {
    io.configPID(kP, kI, kD);
  }

  public void setPivotVelocity(double velocityInput) {
    io.setPivotVelocity(velocityInput);
  }

  public void stopPivot() {
    io.setPivotVelocity(0);
  }

  public double downPos() {
    return io.downPos();
  }

  public boolean isExtenderAlive() {
    return inputs.extenderConnected;
  }

  // * power port fucntion */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
