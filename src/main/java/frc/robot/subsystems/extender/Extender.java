package frc.robot.subsystems.extender;

import edu.wpi.first.units.measure.Velocity;
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

  public double extenderPos() {
    return io.extenderPos();
  }

  public void goToPos(double Pos) {
    io.goToPos(Pos);
  }

  public void setVelocity(double Velocity) {
    io.setVelocity(Velocity);
  }

  public void stop() {
    io.stop();
  }

  // * power port fucntion */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
