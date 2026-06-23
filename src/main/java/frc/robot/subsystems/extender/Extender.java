package frc.robot.subsystems.extender;

import frc.robot.util.RBSISubsystem;

public class Extender extends RBSISubsystem {
  private final ExtenderIO io;
  private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();

  public Extender(ExtenderIO io) {
    this.io = io;
  }

  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
  }

  public void configPID(double kP, double kI, double kD) {
    io.configPID(kP, kI, kD);
  }

  public void setPivotVelocity(double veloccityInput) {
    io.setPivotVelocity(veloccityInput);
  }

  public void stopPivot() {
    io.setPivotVelocity(0);
  }

  public double downPos() {
    return 0.0;
  }

  public boolean isExtenderAlive() {
    return inputs.extenderConected;
  }

  // * power port fucntion */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
