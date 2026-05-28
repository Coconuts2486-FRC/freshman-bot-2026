package frc.robot.subsystems.driver_info;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.imu.Imu;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class CANStatus extends VirtualSubsystem {

  private final Drive drive;
  private final Imu imu;
  private final Extender extender;

  Boolean mainCAN = false;

  /** Constructor */
  public CANStatus(Drive drive, Imu imu, Extender extender) {
    this.drive = drive;
    this.imu = imu;
    this.extender = extender;
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

  @Override
  public void rbsiPeriodic() {

    // figure out if the drive CAN network is alive
    var modules = drive.getModules();

    mainCAN =
        (modules[0].isAlive()
            && modules[1].isAlive()
            && modules[2].isAlive()
            && modules[3].isAlive()
            && imu.isConnected());

    // Logger inputs for each CAN network
    Logger.recordOutput("CAN/MainCAN", mainCAN);

    // Logger inputs for each part of the drive CAN network
    Logger.recordOutput("CAN/PigeonALive", imu.isConnected());
    Logger.recordOutput("CAN/Module1Alive", modules[0].isAlive());
    Logger.recordOutput("CAN/Module2Alive", modules[1].isAlive());
    Logger.recordOutput("CAN/Module3Alive", modules[2].isAlive());
    Logger.recordOutput("CAN/Module4Alive", modules[3].isAlive());

    Logger.recordOutput("CAN/HubActive", FieldState.isHubActive());
    Logger.recordOutput("CAN/MatchTime", DriverStation.getMatchTime());
  }
}
