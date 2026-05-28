// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANBuses;
import frc.robot.Constants.Cameras;
import frc.robot.Constants.OperatorConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathAngleOverride;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.coordinator.Coordinator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveOdometry;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.driver_info.Blinkin;
import frc.robot.subsystems.driver_info.CANStatus;
import frc.robot.subsystems.driver_info.MatchStatus;
import frc.robot.subsystems.extender.Extender;
import frc.robot.subsystems.imu.Imu;
import frc.robot.subsystems.imu.ImuIOSim;
import frc.robot.subsystems.vision.CameraSweepEvaluator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GetJoystickValue;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.RBSICANBusRegistry;
import frc.robot.util.RBSICANHealth;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.DriveStyle;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIPowerMonitor;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  /** Define the Driver and, optionally, the Operator/Co-Driver Controllers */
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  final CommandXboxController driverController = new CommandXboxController(0); // Main Driver

  final Blinkin blinkin = new Blinkin(0);
  final CommandXboxController operatorController = new CommandXboxController(1); // Second Operator
  final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  // These two are needed for the Sweep evaluator for camera FOV simulation
  final CommandJoystick joystick3 = new CommandJoystick(3); //  Joystick for CamersSweepEvaluator
  private final CameraSweepEvaluator sweep;

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controls
  private final Drive m_drivebase;

  private final Extender m_extender;

  // private final Flywheel m_flywheel;

  private final MatchStatus m_matchstatus;

  private boolean elasticOnDriveTab = true;

  // ... Add additional subsystems here (e.g., elevator, arm, etc.)

  // These are "Virtual Subsystems" that report information but have no motors
  private final Imu m_imu;
  private final Vision m_vision;

  @SuppressWarnings("unused")
  private final DriveOdometry m_driveOdometry;

  @SuppressWarnings("unused")
  private final Accelerometer m_accel;

  @SuppressWarnings("unused")
  private final RBSIPowerMonitor m_power;

  @SuppressWarnings("unused")
  private List<RBSICANHealth> m_canHealth;

  @SuppressWarnings("unused")
  private final CANStatus m_canStatus;

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  private final LoggedDashboardChooser<DriveStyle> driveStyle =
      new LoggedDashboardChooser<>("Drive Style");

  private final AutoChooser autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;
  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);
  // EXAMPLE TUNABLE FLYWHEEL SPEED INPUT FROM DASHBOARD

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  public void defineAutoCommands() {

    NamedCommands.registerCommand(
        "Align",
        DriveCommands.fieldRelativeDriveAtAngle(
            m_drivebase, () -> 0.0, () -> 0.0, Coordinator::getRobotAngle));

    NamedCommands.registerCommand(
        "Zero", Commands.runOnce(m_drivebase::zeroHeadingForAlliance, m_drivebase));
  }

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {

    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // Register the CANBus
        RBSICANBusRegistry.initReal(CANBuses.RIO, CANBuses.DRIVE);

        // YAGSL drivebase, get config from deploy directory
        // Get the IMU instance
        m_imu = new Imu(SwerveConstants.kImu.factory.get());

        m_drivebase = new Drive(m_imu);
        m_driveOdometry = new DriveOdometry(m_drivebase, m_imu, m_drivebase.getModules());
        m_vision =
            new Vision(
                m_drivebase, m_drivebase::addVisionMeasurement, buildVisionIOsReal(m_drivebase));
        // m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_accel = new Accelerometer(m_imu);

        m_matchstatus = new MatchStatus(driverController, operatorController, blinkin);

        m_extender = new Extender(null);
        sweep = null;

        break;

      case SIM:
        RBSICANBusRegistry.initSim(CANBuses.RIO, CANBuses.DRIVE);

        m_imu = new Imu(new ImuIOSim());
        m_drivebase = new Drive(m_imu);
        m_driveOdometry = new DriveOdometry(m_drivebase, m_imu, m_drivebase.getModules());
        m_vision =
            new Vision(
                m_drivebase, m_drivebase::addVisionMeasurement, buildVisionIOsSim(m_drivebase));
        // m_flywheel = new Flywheel(new FlywheelIOSim() {});
        m_accel = new Accelerometer(m_imu);
        m_matchstatus = new MatchStatus(driverController, operatorController, blinkin);
        m_extender = new Extender(null);
        // CameraSweepEvaluator (sim-only analysis)
        VisionSystemSim visionSim = new VisionSystemSim("CameraSweepWorld");
        visionSim.addAprilTags(FieldConstants.aprilTagLayout);
        var cams = Cameras.ALL;
        PhotonCameraSim[] simCams = new PhotonCameraSim[cams.length];
        for (int i = 0; i < cams.length; i++) {
          var cfg = cams[i];
          PhotonCamera photonCam = new PhotonCamera(cfg.name());
          PhotonCameraSim camSim = new PhotonCameraSim(photonCam, cfg.simProps());
          visionSim.addCamera(camSim, cfg.robotToCamera());
          simCams[i] = camSim;
        }

        // Create the sweep evaluator (expects two cameras; adapt if you add more later)
        if (simCams.length >= 2) {
          sweep = new CameraSweepEvaluator(visionSim, simCams[0], simCams[1]);
        } else {
          sweep = null; // or throw if you require exactly 2 cameras
        }

        break;

      default:
        // Replayed robot, disable IO implementations
        RBSICANBusRegistry.initSim(CANBuses.RIO, CANBuses.DRIVE);
        m_imu = new Imu(new ImuIOSim() {});
        m_drivebase = new Drive(m_imu);
        m_driveOdometry = new DriveOdometry(m_drivebase, m_imu, m_drivebase.getModules());
        m_vision =
            new Vision(
                m_drivebase, m_drivebase::addVisionMeasurement, buildVisionIOsReplay(m_drivebase));

        // m_flywheel = new Flywheel(new FlywheelIO() {});
        m_accel = new Accelerometer(m_imu);
        sweep = null;
        m_matchstatus = new MatchStatus(driverController, operatorController, blinkin);
        m_extender = new Extender(null);
        break;
    }

    // Init all CAN busses specified in the `Constants.CANBuses` class
    RBSICANBusRegistry.initReal(Constants.CANBuses.ALL);
    m_canHealth = Arrays.stream(Constants.CANBuses.ALL).map(RBSICANHealth::new).toList();
    m_canStatus = new CANStatus(m_drivebase, m_imu, null);

    // In addition to the initial battery capacity from the Dashbaord, ``RBSIPowerMonitor`` takes
    // all the non-drivebase subsystems for which you wish to have power monitoring; DO NOT
    // include ``m_drivebase``, as that is automatically monitored.
    m_power = new RBSIPowerMonitor(batteryCapacity);

    // Build the coordinator

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case MANUAL:
        // This is where the "Leave Auto" will go
        // ...
        // Set the others to null
        autoChooserPathPlanner = null;
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        PPHolonomicDriveController.setRotationTargetOverride(PathAngleOverride::getOverride);

        // Set the others to null
        autoChooserChoreo = null;
        autoFactoryChoreo = null;

        // Set up the logging callback
        PathPlannerLogging.setLogActivePathCallback(
            (List<Pose2d> activePath) -> {
              Logger.recordOutput("PathPlanner/ActivePath", activePath.toArray(new Pose2d[0]));
            });

        PathPlannerLogging.setLogCurrentPoseCallback(
            (Pose2d currentPose) -> {
              Logger.recordOutput("PathPlanner/CurrentPose", currentPose);
            });

        PathPlannerLogging.setLogTargetPoseCallback(
            (Pose2d targetPose) -> {
              Logger.recordOutput("PathPlanner/TargetPose", targetPose);
            });

        break;

      case CHOREO:
        autoFactoryChoreo =
            new AutoFactory(
                m_drivebase::getPose, // A function that returns the current robot pose
                m_drivebase::resetOdometry, // A function that resets the current robot pose to the
                // provided Pose2d
                m_drivebase::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                m_drivebase // The drive subsystem
                );
        autoChooserChoreo = new AutoChooser();
        autoChooserChoreo.addRoutine("twoPieceAuto", this::twoPieceAuto);
        // Set the others to null
        autoChooserPathPlanner = null;
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Get drive style from the Dashboard Chooser
    driveStyle.addDefaultOption("TANK", DriveStyle.TANK);
    driveStyle.addOption("GAMER", DriveStyle.GAMER);

    // Define SysIs Routines
    definesysIdRoutines();
    // Configure the button and trigger bindings
    configureBindings();
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // Send the proper joystick input based on driver preference -- Set this in `Constants.java`
    GetJoystickValue driveStickY;
    GetJoystickValue driveStickX;
    GetJoystickValue turnStickX;
    // OPTIONAL: Use the DashboardChooser rather than the Constants file for Drive Style
    // switch (driveStyle.get()) {
    switch (OperatorConstants.kDriveStyle) {
      case GAMER:
        driveStickY = driverController::getRightY;
        driveStickX = driverController::getRightX;
        turnStickX = driverController::getLeftX;
        break;
      default: // Includes case TANK
        driveStickY = driverController::getLeftY;
        driveStickX = driverController::getLeftX;
        turnStickX = driverController::getRightX;
    }

    // =======================================================================
    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driveStickY.value(),
            () -> -driveStickX.value(),
            () -> -turnStickX.value()));

    // =======================================================================
    // Set DEFAULT COMMANDS for subsystems

    // driver controls

    // Press X button --> Stop with wheels in X-Lock position
    driverController.x().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));

    // Press Start button --> Manually Re-Zero the Gyro
    driverController
        .start()
        .onTrue(
            Commands.runOnce(m_drivebase::zeroHeadingForAlliance, m_drivebase)
                .ignoringDisable(true));

    // oter feeding

    // auto aim - turn only, driver keeps translational control

    driverController
        .povUp()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(-16), Units.inchesToMeters(0), 0));
                },
                m_drivebase));

    driverController
        .povDown()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(16), Units.inchesToMeters(0), 0));
                },
                m_drivebase));

    // micro driving controls
    driverController
        .povRight()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(0.), Units.inchesToMeters(-20.0), 0.));
                },
                m_drivebase));

    driverController
        .povLeft()
        .whileTrue(
            Commands.run(
                () -> {
                  m_drivebase.runVelocity(
                      new ChassisSpeeds(Units.inchesToMeters(0.), Units.inchesToMeters(20.0), 0.));
                },
                m_drivebase));
    //
    // ===============================================================================

    // Co driver controls

    // Press start button --> switch elastic tab
    operatorController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Elastic.selectTab(1);
                }));

    operatorController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  Elastic.selectTab(0);
                }));

    // ============================================================================================================================
    // sim controls
    if (Constants.getMode() == Mode.SIM) {
      // IN SIMULATION ONLY:
      // Double-press the A button on Joystick3 to run the CameraSweepEvaluator
      // Use WPILib's built-in double-press binding
      joystick3
          .button(1)
          .multiPress(2, 0.2)
          .onTrue(
              Commands.runOnce(
                  () -> {
                    try {
                      sweep.runFullSweep(
                          Filesystem.getOperatingDirectory()
                              .toPath()
                              .resolve("camera_sweep.csv")
                              .toString());
                    } catch (Exception e) {
                      e.printStackTrace();
                    }
                  }));
    }
  }

  /**
   * Use this to pass the MANUAL SHOOT FUEL command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getManualAuto() {
    // NOTE:
    //
    // For teams not using PathPlanner, this auto may be used to simply shoot the pre-loaded fuel
    // into the HUB during AUTO.  Since shooters are beyond the scope of Az-RBSI, you will have to
    // write your own command and call it here.

    // Replace Commands.none() with your command that shoots fuel into the HUB.
    return Commands.none();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommandChoreo() {
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooserChoreo);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooserChoreo.selectedCommandScheduler());
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive =
        Constants.getAprilTagLayoutType() != AprilTagLayoutType.REBUILT_WELDED;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /** Drivetrain getter method for use with Robot.java */
  public Drive getDrivebase() {
    return m_drivebase;
  }

  /** Vision getter method for use with Robot.java */
  public Vision getVision() {
    return m_vision;
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == AutoType.PATHPLANNER) {
      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Example Flywheel SysId Characterization

    }
  }

  // Vision Factories
  // Vision Factories (REAL)
  private VisionIO[] buildVisionIOsReal(Drive drive) {
    return switch (Constants.getVisionType()) {
      case PHOTON ->
          Arrays.stream(Constants.Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOPhotonVision(c.name(), c.robotToCamera()))
              .toArray(VisionIO[]::new);

      case LIMELIGHT ->
          Arrays.stream(Constants.Cameras.ALL)
              .map(c -> (VisionIO) new VisionIOLimelight(c.name(), drive::getHeading))
              .toArray(VisionIO[]::new);

      case NONE -> new VisionIO[] {}; // recommended: no cameras
    };
  }

  // Vision Factories (SIM)
  private VisionIO[] buildVisionIOsSim(Drive drive) {
    var cams = Constants.Cameras.ALL;
    VisionIO[] ios = new VisionIO[cams.length];
    for (int i = 0; i < cams.length; i++) {
      var cfg = cams[i];
      ios[i] = new VisionIOPhotonVisionSim(cfg.name(), cfg.robotToCamera(), drive::getPose);
    }
    return ios;
  }

  // Vision Factories (REPLAY)
  private VisionIO[] buildVisionIOsReplay(Drive drive) {
    var cams = Constants.Cameras.ALL;

    VisionIO[] ios = new VisionIO[cams.length];
    for (int i = 0; i < cams.length; i++) {
      ios[i] =
          new VisionIO() {
            @Override
            public void updateInputs(VisionIOInputs inputs) {
              // Intentionally empty.
              // Logger.processInputs("Vision/Camera" + i, inputs) will populate these from the log.
            }
          };
    }
    return ios;
  }

  /**
   * Example Choreo auto command
   *
   * <p>NOTE: This would normally be in a spearate file.
   */
  private AutoRoutine twoPieceAuto() {
    AutoRoutine routine = autoFactoryChoreo.newRoutine("twoPieceAuto");

    // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
    AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(pickupTraj.resetOdometry(), pickupTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(scoreTraj.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return routine;
  }
}
