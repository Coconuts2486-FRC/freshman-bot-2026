package frc.robot.subsystems.extender;

import edu.wpi.first.units.measure.Velocity;
import frc.robot.util.RBSISubsystem;

public class extender extends RBSISubsystem {
  private final extenderIO io;
  private final extenderIOInputsAutoLogged inputs = new extenderIOInputsAutoLogged();

  public extender(extenderIO io) {
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
}
