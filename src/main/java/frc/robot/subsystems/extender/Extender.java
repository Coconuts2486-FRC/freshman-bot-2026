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

  public void extendBlocker() {
    io.extendBlocker();
  }

  public void retractBlocker() {
    io.retractBlocker();
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
