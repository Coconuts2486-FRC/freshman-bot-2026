package frc.robot.subsystems.extender;

import com.ctre.phoenix.Logger;
import frc.robot.util.RBSISubsystem;

public class extender extends RBSISubsystem {
  private final extenderIO io;
  private final extenderIOInputsAutoLogged inputs = new extenderIOInputsAutoLogged();

  public extender(extenderIO io) {
    this.io = io;
  }

  @Override
  protected void rbsiPeriodic() {
    io.updateIntputs(inputs);
    Logger.processInputs("extender", inputs);
  }
}
