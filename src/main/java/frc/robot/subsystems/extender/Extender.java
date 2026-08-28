package frc.robot.subsystems.extender;

import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Extender extends RBSISubsystem {
  private final ExtenderIO io;
  private final ExtenderIOInputsAutoLogged inputs = new ExtenderIOInputsAutoLogged();
  private final Alert compressorDisconnectedAlert =
      new Alert("PCM reports that the compressor is disconnected.", AlertType.ERROR);
  private final Alert compressorShortedAlert =
      new Alert("PCM reports that the compressor output is shorted.", AlertType.ERROR);
  private final Alert compressorOvercurrentAlert =
      new Alert("PCM reports compressor overcurrent.", AlertType.WARNING);
  private final Alert solenoidVoltageAlert =
      new Alert("PCM reports a solenoid voltage fault.", AlertType.WARNING);

  public Extender(ExtenderIO io) {
    this.io = io;
  }

  @Override
  protected void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Extender", inputs);
    compressorDisconnectedAlert.set(inputs.compressorNotConnectedFault);
    compressorShortedAlert.set(inputs.compressorShortedFault);
    compressorOvercurrentAlert.set(inputs.compressorCurrentTooHighFault);
    solenoidVoltageAlert.set(inputs.solenoidVoltageFault);
  }

  public void extendBlocker() {
    io.extendBlocker();
  }

  public void retractBlocker() {
    io.retractBlocker();
  }

  /** Toggles the blocker from the PCM's currently reported solenoid output state. */
  public void toggleBlocker() {
    if (inputs.blockerSolenoidEnabled) {
      retractBlocker();
    } else {
      extendBlocker();
    }
  }

  public boolean isExtenderAlive() {
    return inputs.compressorConnected;
  }

  // * power port fucntion */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
