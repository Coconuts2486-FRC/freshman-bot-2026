package frc.robot.generated;

import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public final class TunerFactory {
  public static final TunerView INSTANCE = create();

  private static TunerView create() {
    // Debugging
    Logger.recordOutput("Drive/EncoderOffsets/RobotType", Constants.getRobot());

    // Return the proper view into TunerConstants
    return switch (Constants.getRobot()) {
      case PINCHY -> new PinchyTunerView();
      case GEORGE -> new GeorgeTunerView();
      case COMPBOT -> new CompbotTunerView();
      default -> new CompbotTunerView();
    };
  }
}
