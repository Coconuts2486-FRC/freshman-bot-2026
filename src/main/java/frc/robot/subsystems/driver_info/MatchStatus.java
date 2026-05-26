package frc.robot.subsystems.driver_info;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FieldState;
import frc.robot.subsystems.driver_info.Blinkin.LEDState;
import frc.robot.util.VirtualSubsystem;

public class MatchStatus extends VirtualSubsystem {

  private final CommandXboxController driver;
  private final CommandXboxController coDriver;
  private final Blinkin blinkin;
  Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  private boolean isRumbling = false;

  /** Constructor */
  public MatchStatus(
      CommandXboxController driver, CommandXboxController coDriver, Blinkin blinkin) {
    this.driver = driver;
    this.coDriver = coDriver;
    this.blinkin = blinkin;
  }

  /**
   * Make both controllers rumble
   *
   * @param strength Rumble strength
   */
  public void rumble(double strength) {
    if (!isRumbling) {
      driver.setRumble(RumbleType.kBothRumble, strength);
      coDriver.setRumble(RumbleType.kBothRumble, strength);
      isRumbling = true;
    }
  }

  /** Make both controllers stop rumbling */
  public void stopRumble() {
    if (isRumbling) {
      driver.setRumble(RumbleType.kBothRumble, 0);
      coDriver.setRumble(RumbleType.kBothRumble, 0);
      isRumbling = false;
    }
  }

  /**
   * Priority value for this virtual subsystem
   *
   * <p>See `frc.robot.util.VirtualSubsystem` for a description of the suggested values for various
   * virtual subsystems.
   */
  @Override
  protected int getPeriodPriority() {
    // Low-priority status system
    return 10;
  }

  /** Periodic function */
  @Override
  public void rbsiPeriodic() {

    // Do these deeper calls once, and use the values for the rest of this loop
    double matchTime = DriverStation.getMatchTime();
    if (DriverStation.isAutonomous()) {
      // If Auto, don't do anything for this
      blinkin.setState(LEDState.AUTONOMOUS);
      return;
    }

    // Alliance-Specific HUB Activation / Deactivation
    // Rumble & return
    if (FieldState.wonAuto == alliance) {

      blinkin.setState(LEDState.TELEOP_RED);
      if (matchTime < 135 && matchTime > 130) {
        // Deactivating HUB
        rumble(0.5);
        blinkin.setState(LEDState.DEACTIVE);
        return;
      }
      if (matchTime < 110 && matchTime > 105) {
        // Activating HUB
        rumble(0.5);
        blinkin.setState(LEDState.ACTIVE);
        return;
      }
      if (matchTime < 85 && matchTime > 80) {
        // Deactivating HUB
        blinkin.setState(LEDState.DEACTIVE);
        rumble(0.5);
        return;
      }
      if (matchTime < 60 && matchTime > 55) {
        // Activating HUB
        rumble(0.5);
        blinkin.setState(LEDState.ACTIVE);
        return;
      }

    } else if (FieldState.wonAuto != alliance) {

      blinkin.setState(LEDState.TELEOP_BLUE);
      if (matchTime < 110 && matchTime > 105) {
        // Deactivating HUB
        blinkin.setState(LEDState.DEACTIVE);
        rumble(0.5);
        return;
      }
      if (matchTime < 85 && matchTime > 80) {
        // Activating HUB
        blinkin.setState(LEDState.ACTIVE);
        rumble(0.5);
        return;
      }
      if (matchTime < 60 && matchTime > 55) {
        // Deactivating HUB
        blinkin.setState(LEDState.DEACTIVE);
        rumble(0.5);
        return;
      }
      if (matchTime < 35 && matchTime > 31) {
        // Activating HUB
        blinkin.setState(LEDState.ACTIVE);
        rumble(0.5);
        return;
      }
    }

    // Endgame Rumble -- same regardless
    if (matchTime < 31 && matchTime > 30) {
      rumble(0.25);
      blinkin.setState(LEDState.ENDGAME);
      return;
    }
    // Rumble every second during the last 10 seconds of the match
    if (matchTime < 10 && isUpperHalfSecond(matchTime)) {
      rumble(0.25);
      return;
    }

    // If we get all the way here, stop rumble!
    stopRumble(); // if no value won for who won auto the controllers will never vibrate
  }

  /**
   * Check that the time is in the "upper half" of the second
   *
   * @param time The matchTime
   */
  private boolean isUpperHalfSecond(double time) {

    // The midpoint is halfway between the floor and ceiling
    double midpoint = (double) (Math.floor(time) + Math.ceil(time)) / 2.0;
    return time > midpoint;
  }
}
