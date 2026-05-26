package frc.robot.subsystems.driver_info;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class CANStatus extends VirtualSubsystem {

  private final Drive drive;
  private final Imu imu;

  private final Intake intake;
  private final Feeder feeder;
  private final Rollers rollers;
  private final Shooter shooter;
  private final Indexer indexer;

  Boolean driveCAN = false;
  Boolean mainCAN = false;

  /** Constructor */
  public CANStatus(
      Drive drive,
      Imu imu,
      Intake intake,
      Feeder feeder,
      Rollers rollers,
      Shooter shooter,
      Indexer indexer) {
    this.drive = drive;
    this.imu = imu;

    this.intake = intake;
    this.feeder = feeder;
    this.rollers = rollers;
    this.shooter = shooter;
    this.indexer = indexer;
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

    driveCAN =
        (modules[0].isAlive()
            && modules[1].isAlive()
            && modules[2].isAlive()
            && modules[3].isAlive()
            && imu.isConnected());

    mainCAN =
        (intake.pivotAlive()
            && feeder.isFeederAlive()
            && Math.abs(intake.getPivotPosition()) > 0.0
            && rollers.isRollersAlive()
            && shooter.leaderAlive()
            && shooter.followerAlive()
            && indexer.indexerAlive());

    // Logger inputs for each CAN network
    Logger.recordOutput("CAN/DriveCAN", driveCAN);
    Logger.recordOutput("CAN/MainCAN", mainCAN);

    // Logger inputs for each part of the drive CAN network
    Logger.recordOutput("CAN/PigeonALive", imu.isConnected());
    Logger.recordOutput("CAN/Module1Alive", modules[0].isAlive());
    Logger.recordOutput("CAN/Module2Alive", modules[1].isAlive());
    Logger.recordOutput("CAN/Module3Alive", modules[2].isAlive());
    Logger.recordOutput("CAN/Module4Alive", modules[3].isAlive());

    // logger inputs for each part of the main CAN network
    Logger.recordOutput("CAN/IntakePivotAlive", intake.pivotAlive());
    Logger.recordOutput("CAN/FeederAlive", feeder.isFeederAlive());
    Logger.recordOutput("CAN/IntakeCancoderAlive", Math.abs(intake.getPivotPosition()) > 0.0);
    Logger.recordOutput("CAN/IntakeRollersAlive", rollers.isRollersAlive());
    Logger.recordOutput("CAN/FlywheelLeaderAlive", shooter.leaderAlive());
    Logger.recordOutput("CAN/FlywheelFollowerAlive", shooter.followerAlive());
    Logger.recordOutput("CAN/IndexerAlive", indexer.indexerAlive());

    Logger.recordOutput("CAN/HubActive", FieldState.isHubActive());
    Logger.recordOutput("CAN/MatchTime", DriverStation.getMatchTime());

    Logger.recordOutput("CAN/intakePos", intake.getPivotPosition());
    Logger.recordOutput("CAN/IntakeDeployed", intake.isIntakeExtended());
  }
}
