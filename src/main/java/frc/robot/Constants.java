// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
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

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.Alert;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.CTREPro;
import frc.robot.util.RBSIEnum.DriveStyle;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIEnum.MotorIdleMode;
import frc.robot.util.RBSIEnum.SwerveType;
import frc.robot.util.RBSIEnum.VisionType;
import frc.robot.util.RobotDeviceId;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.SimCameraProperties;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /***************************************************************************/
  /**
   * Define the various multiple robots that use this same code (e.g., COMPBOT, DEVBOT, SIMBOT,
   * etc.) and the operating modes of the code (REAL, SIM, or REPLAY)
   */
  private static RobotType robotType = RobotType.COMPBOT;

  // Define swerve, auto, and vision types being used
  // NOTE: Only PHOENIX6 swerve base has been tested at this point!!!
  //       If you have a swerve base with non-CTRE compoments, use YAGSL
  //       under strict caveat emptor -- and submit any error and bugfixes
  //       via GitHub issues.
  private static SwerveType swerveType = SwerveType.PHOENIX6; // PHOENIX6, YAGSL
  private static CTREPro phoenixPro = CTREPro.LICENSED; // LICENSED, UNLICENSED
  private static AutoType autoType = AutoType.PATHPLANNER; // MANUAL, PATHPLANNER, CHOREO
  private static VisionType visionType = VisionType.PHOTON; // PHOTON, LIMELIGHT, NONE

  /** Enumerate the robot types (name your robots here) */
  public static enum RobotType {
    GEORGE, // Development / Alpha / Practice Bot
    PINCHY, // Development / Alpha / Practice Bot
    COMPBOT, // Competition robot
    SIMBOT // Simulated robot
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }

  /** Disable the Hardware Abstraction Layer, if requested */
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /***************************************************************************/
  /* The remainder of this file contains physical and/or software constants for the various subsystems of the robot */

  /** General Constants **************************************************** */
  public static final double loopPeriodSecs = 0.02;

  public static final boolean tuningMode = true;

  public static final double G_TO_MPS2 = 9.80665; // Gravitational acceleration in m/s/s

  /************************************************************************* */
  /** Physical Constants for Robot Operation ******************************* */
  public static final class RobotConstants {

    public static final Mass kRobotMass = Pounds.of(100.);
    public static final Matter kChassis =
        new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), kRobotMass.in(Kilograms));
    // Robot moment of intertial; this can be obtained from a CAD model of your drivetrain. Usually,
    // this is between 3 and 8 kg*m^2.
    public static final double kRobotMOI = 6.8;

    // Wheel coefficient of friction
    public static final double kWheelCOF = 1.2;

    // Maximum torque applied by wheel
    // Kraken X60 stall torque ~7.09 Nm; MK4i L3 gear ratio 6.12:1
    public static final double kMaxWheelTorque = 43.4; // Nm

    // Insert here the orientation (CCW == +) of the Rio and IMU from the robot
    // An angle of "0." means the x-y-z markings on the device match the robot's intrinsic reference
    //   frame.
    public static final Rotation3d kRioOrientation =
        switch (getRobot()) {
          case COMPBOT -> new Rotation3d(0, 0, -90);
          case GEORGE -> Rotation3d.kZero;
          default -> Rotation3d.kZero;
        };
    // IMU can be one of Pigeon2 or NavX
    public static final Rotation3d kIMUOrientation =
        switch (getRobot()) {
          case COMPBOT -> Rotation3d.kZero;
          case GEORGE -> Rotation3d.kZero;
          default -> Rotation3d.kZero;
        };
  }

  /************************************************************************* */
  /** Power Distribution Constants ***************************************** */
  public static final class PowerConstants {

    // Power Distribution Module Configuration
    public static final PowerDistribution.ModuleType kPDMType = PowerDistribution.ModuleType.kRev;
    public static final int kPDMCANid = 1;

    // Current Limits
    public static final double kTotalMaxCurrent = 120.;
    public static final double kMotorPortMaxCurrent = 40.;
    public static final double kSmallPortMaxCurrent = 20.;

    // Brownout voltage levels
    public static final double kVoltageWarning = 7.5;
    public static final double kVoltageLimiting = 7.0;
    public static final double kVoltageCritical = 6.5;
  }

  /************************************************************************* */
  /** List of Robot CAN Busses ********************************************* */
  public static final class CANBuses {
    public static final String RIO = "rio";
    public static final String DRIVE = "DriveTrain";

    public static final String[] ALL = {RIO, DRIVE};
  }

  /************************************************************************* */
  /** List of Robot Device CAN and Power Distribution Circuit IDs ********** */
  public static class RobotDevices {

    /* DRIVETRAIN CAN DEVICE IDS */
    // Input the correct Power Distribution Module port for each motor!!!!
    // NOTE: The CAN ID's are set in the Swerve Generator (Phoenix Tuner or YAGSL)

    // Front Left
    public static final RobotDeviceId FL_DRIVE =
        new RobotDeviceId(SwerveConstants.kFLDriveMotorId, SwerveConstants.kFLDriveCanbus, 11);
    public static final RobotDeviceId FL_ROTATION =
        new RobotDeviceId(SwerveConstants.kFLSteerMotorId, SwerveConstants.kFLSteerCanbus, 10);
    public static final RobotDeviceId FL_CANCODER =
        new RobotDeviceId(SwerveConstants.kFLEncoderId, SwerveConstants.kFLEncoderCanbus, null);
    // Front Right
    public static final RobotDeviceId FR_DRIVE =
        new RobotDeviceId(SwerveConstants.kFRDriveMotorId, SwerveConstants.kFRDriveCanbus, 3);
    public static final RobotDeviceId FR_ROTATION =
        new RobotDeviceId(SwerveConstants.kFRSteerMotorId, SwerveConstants.kFRSteerCanbus, 2);
    public static final RobotDeviceId FR_CANCODER =
        new RobotDeviceId(SwerveConstants.kFREncoderId, SwerveConstants.kFREncoderCanbus, 1);
    // Back Left
    public static final RobotDeviceId BL_DRIVE =
        new RobotDeviceId(SwerveConstants.kBLDriveMotorId, SwerveConstants.kBLDriveCanbus, 14);
    public static final RobotDeviceId BL_ROTATION =
        new RobotDeviceId(SwerveConstants.kBLSteerMotorId, SwerveConstants.kBLSteerCanbus, 15);
    public static final RobotDeviceId BL_CANCODER =
        new RobotDeviceId(SwerveConstants.kBLEncoderId, SwerveConstants.kBLEncoderCanbus, null);
    // Back Right
    public static final RobotDeviceId BR_DRIVE =
        new RobotDeviceId(SwerveConstants.kBRDriveMotorId, SwerveConstants.kBRSteerCanbus, 18);
    public static final RobotDeviceId BR_ROTATION =
        new RobotDeviceId(SwerveConstants.kBRSteerMotorId, SwerveConstants.kBRSteerCanbus, 17);
    public static final RobotDeviceId BR_CANCODER =
        new RobotDeviceId(SwerveConstants.kBREncoderId, SwerveConstants.kBREncoderCanbus, null);
    // Pigeon
    public static final RobotDeviceId PIGEON =
        new RobotDeviceId(SwerveConstants.kPigeonId, SwerveConstants.kCANbusName, null);

    /* SUBSYSTEM CAN DEVICE IDS */
    // This is where mechanism subsystem devices are defined (Including ID, bus, and power port)
    public static final RobotDeviceId SHOOTER_LEADER = new RobotDeviceId(26, CANBuses.RIO, 6);
    public static final RobotDeviceId SHOOTER_FOLLOWER = new RobotDeviceId(25, CANBuses.RIO, 7);

    public static final RobotDeviceId INTAKE_PIVOT = new RobotDeviceId(11, CANBuses.RIO, 19);
    public static final RobotDeviceId INTAKE_ROLLER = new RobotDeviceId(13, CANBuses.RIO, 0);
    public static final RobotDeviceId INTAKE_ENCODER = new RobotDeviceId(43, CANBuses.RIO, null);

    public static final RobotDeviceId INDEXER_ROLLER = new RobotDeviceId(15, CANBuses.RIO, 9);

    public static final RobotDeviceId FEEDER_ROLLER = new RobotDeviceId(20, CANBuses.RIO, 13);

    public static final RobotDeviceId TURRET_POINTER = new RobotDeviceId(31, CANBuses.RIO, 5);
    public static final RobotDeviceId TURRET_ENCODER = new RobotDeviceId(42, CANBuses.RIO, null);

    public static final RobotDeviceId CLIMB_MOTOR = new RobotDeviceId(20, CANBuses.DRIVE, 16);
    public static final RobotDeviceId CLIMB_ENCODER = new RobotDeviceId(21, CANBuses.DRIVE, null);

    public static final RobotDeviceId EXTENDER_MOTOR = new RobotDeviceId(16, 2);
    public static final RobotDeviceId EXTENDER_ENCODER = new RobotDeviceId(17, null);

    /* BEAM BREAK and/or LIMIT SWITCH DIO CHANNELS */
    // This is where digital I/O feedback devices are defined
    // Example:
    // public static final int ELEVATOR_BOTTOM_LIMIT = 3;
    public static final int TURRET_MAGHOME = 0;
    public static final int INTAKE_RELEASE = 9;

    /* LINEAR SERVO PWM CHANNELS */
    // This is where PWM-controlled devices (actuators, servos, pneumatics, etc.)
    // are defined
    // Example:
    // public static final int INTAKE_SERVO = 0;
    public static final int LEDS = 0;
  }

  /************************************************************************* */
  /** Operator Constants *************************************************** */
  public static class OperatorConstants {

    // Joystick Functions
    // Set to TANK for Drive = Left Stick, Turn = Right Stick;
    // Set to GAMER for Drive = Right Stick, Turn = Left Stick;
    // NOTE: Intrepid programmers can turn this into a Dashboard-settable value
    public static final DriveStyle kDriveStyle = DriveStyle.GAMER; // TANK, GAMER

    // Joystick Deadbands
    public static final double kDeadband = 0.1;
    public static final double kTurnConstant = 6;

    // Joystick slew rate limiters to smooth erratic joystick motions, measured in units per second
    public static final double kJoystickSlewLimit = 0.5;

    // Override and Console Toggle Switches
    // Assumes this controller: https://www.amazon.com/gp/product/B00UUROWWK
    // Example from:
    // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/72
    public static final int DRIVER_SWITCH_0 = 1;
    public static final int DRIVER_SWITCH_1 = 2;
    public static final int DRIVER_SWITCH_2 = 3;

    public static final int OPERATOR_SWITCH_0 = 8;
    public static final int OPERATOR_SWITCH_1 = 9;
    public static final int OPERATOR_SWITCH_2 = 10;
    public static final int OPERATOR_SWITCH_3 = 11;
    public static final int OPERATOR_SWITCH_4 = 12;

    public static final int[] MULTI_TOGGLE = {4, 5};
  }

  /************************************************************************* */
  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Theoretical free speed (m/s) at 12v applied output;
    // IMPORTANT: Follow the AdvantageKit instructions for measuring the ACTUAL maximum linear speed
    // of YOUR ROBOT, and replace the estimate here with your measured value!
    public static final double kMaxLinearSpeed = Meters.of(5.0).in(Meters);

    // Slip Current -- the current draw when the wheels start to slip
    // Measure this against a wall.  CHECK WITH THE CARPET AT AN ACTUAL EVENT!!!
    public static final double kSlipCurrent = 80; // Amps

    // Characterized Wheel Radius (using the "Drive Wheel Radius Characterization" auto routine)
    public static final double kWheelRadiusMeters = Inches.of(1.900).in(Meters);

    // Maximum chassis accelerations desired for robot motion  -- metric / radians
    // TODO: Compute the maximum linear acceleration given the PHYSICS of the ROBOT!
    public static final double kMaxLinearAccel = 4.0; // m/s/s

    // For Profiled PID Motion -- NEED TUNING!
    // Used in a variety of contexts, including PathPlanner and AutoPilot
    // Chassis (not module) across-the-field strafing motion
    public static final double kPStrafe = 8; // 12.5
    public static final double kIStrafe = 0.0;
    public static final double kDStrafe = 0.0;
    // Chassis (not module) solid-body rotation
    public static final double kPSPin = 8; // 13
    public static final double kISPin = 0.0;
    public static final double kDSpin = 0.0;

    // Hold time on motor brakes when disabled
    public static final double kWheelLockTime = 10; // seconds

    // SysID characterization constants
    public static final double kMaxV = 12.0; // Max volts
    public static final double kDelay = 3.0; // seconds
    public static final double kQuasiTimeout = 5.0; // seconds
    public static final double kDynamicTimeout = 3.0; // seconds

    // Drive motor open-loop and closed-loop ramp periods for current smoothing
    //   Time from from 0 -> full duty
    public static final double kDriveClosedLoopRampPeriod = 0.15; // seconds
    public static final double kDriveOpenLoopRampPeriod = 0.25; // seconds

    public static final double kOptimalVoltage = 12.0; // Volts
    public static final double kNominalFFVolts = 2.0; // Volts

    // Default TalonFX Gains (Replaces what's in Phoenix X's Tuner Constants)
    // NOTE: Default values from 6328's 2025 Public Code
    //
    // IMPORTANT:: These values are valid only for CTRE LICENSED operation!!
    //             Adjust these downward until your modules behave correctly
    public static final double kDriveP = 50.0;
    public static final double kDriveD = 0.03;
    public static final double kDriveV = 0.9;
    public static final double kDriveA = 0.1;
    public static final double kDriveS = 3.5;
    public static final double kDriveT =
        SwerveConstants.kDriveGearRatio / DCMotor.getKrakenX60Foc(1).KtNMPerAmp;
    public static final double kSteerP = 500.0;
    public static final double kSteerD = 20.0;
    public static final double kSteerS = 2.0;

    // Odometry-related constants ==================================
    public static final double kHistorySize = 1.5; // seconds
    // How aggressively to pull pose toward vision while DISABLED.
    // 0.10 = gentle, 0.25 = fairly quick, 1.0 = full snap.
    public static final double kDisabledVisionBlendAlpha = 0.15;
    // Optional: ignore obviously insane measurements while disabled.
    public static final double kDisabledVisionMaxJumpM = 2.0; // meters
    public static final double kDisabledVisionMaxJumpRad = Units.degreesToRadians(20.0);
    public static final double kDisabledVisionStale = 0.75; // seconds

    // Coast window config
    public static final double kDisabledCoastSeconds = 5.0;

    // "Stationary" detection config (tune)
    public static final double kStationaryMaxWheelDeltaM = 0.002; // 2mm per loop
    public static final double kStationaryMaxYawRateRadPerSec = 0.05; // ~3 deg/s
    public static final int kStationaryLoopsToEndCoast = 10; // ~0.20s @ 20ms
    public static final double kDisabledVisionIgnoreAfterDisableSec = 0.25; // 250ms
  }

  /************************************************************************* */
  /** Example Flywheel Mechanism Constants ********************************* */
  public static final class ShooterConstants {

    public static final Transform3d kShooterTransform =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.5), 0.0, Units.inchesToMeters(20.0)),
            Rotation3d.kZero);

    // Mechanism idle mode
    public static final MotorIdleMode kShooterIdleMode = MotorIdleMode.COAST; // BRAKE, COAST

    // Mechanism motor gear ratio
    public static final double kShooterGearRatio = 24.0 / 18.0;
    public static final double kFlywheelCircumfrence = Math.PI * Units.inchesToMeters(4.0); // πD
    public static final double kTimeOfFlight = 1.0;
    public static final double kShotAngle = Units.degreesToRadians(65.0);

    // Flywheel motor open-loop and closed-loop ramp periods for current smoothing
    //   Time from from 0 -> full duty
    public static final double kShooterClosedLoopRampPeriod = 0.15; // seconds
    public static final double kShooterOpenLoopRampPeriod = 0.25; // seconds

    // MODE == REAL / REPLAY
    // Feedforward constants
    public static final double kSreal = 0.27;
    public static final double kVreal = 0.1;
    public static final double kAreal = 0.002;
    // Feedback (PID) constants
    public static final double kPreal = 4.0;
    public static final double kDreal = 0.0;

    // MODE == SIM
    // Feedforward constants
    public static final double kSsim = 0.0;
    public static final double kVsim = 0.03;
    public static final double kAsim = 0.0;
    // Feedback (PID) constants
    public static final double kPsim = 0.0;
    public static final double kDsim = 0.0;

    // Fuel trajectory Constants
    public static final double kThetaRad = Units.degreesToRadians(55.0); // fixed elevation
    public static final double kApexClearanceMeters = 0.5; // h_c
    public static final double kG = 9.81;

    // Numerical Trajectory Solving Parameters
    public static final double kV0Tol = 1e-6; // m/s
    public static final int kMaxBisectionIters = 80;
    public static final double kMinBracket = 0.1; // m/s
    public static final double kMaxV0Search = 100.0; // m/s safety cap

    public static final LoggedTunableNumber kTestShooterSpeed =
        new LoggedTunableNumber("Tuning/Shooter", 0.0);
    public static final LoggedTunableNumber kTestFeederSpeed =
        new LoggedTunableNumber("Tuning/Feeder", 0.0);
  }

  /** Intake Mechanism Constants ******************************************* */
  public static final class IntakeConstants {

    // public static final AngularVelocity kMaxPivotSpeed = RotationsPerSecond.of(106.3);

    // Mechanism idle mode
    public static final MotorIdleMode kIntakeIdleMode = MotorIdleMode.COAST; // BRAKE, COAST

    // Pivot angle positions
    public static final double dropPosition = 0.72;
    public static final double storedAngle = 0.91;
    public static final double lowerPosition = 0.62;

    // Pivot gear ratio
    public static final double kPivotGearRatio = 25.0 * 54.0 / 16.0;

    // PID Values for the intake pivot
    public static final LoggedTunableNumber kp = new LoggedTunableNumber("Intake/kp", 0.75);
    public static final double ki = 0;
    public static final double kd = 0;

    public static final LoggedTunableNumber maxAcel = new LoggedTunableNumber("Intake/MaxAccel", 5);
    public static final LoggedTunableNumber maxVelocity =
        new LoggedTunableNumber("Intake/maxVel", 5);

    // Intake rollers constats
    public static final double kRollerPrimitiveSpeed = 0.55; // 0.65
    public static final double kRollersRPM = 4500.0;

    // INTAKE ROLLERS PIDS
    // MODE == REAL / REPLAY
    // Feedforward constants
    public static final double kSreal = 0.27;
    public static final double kVreal = 0.1;
    public static final double kAreal = 0.002;
    // Feedback (PID) constants
    public static final double kPreal = 4.0;
    public static final double kDreal = 0.0;

    // MODE == SIM
    // Feedforward constants
    public static final double kSsim = 0.0;
    public static final double kVsim = 0.03;
    public static final double kAsim = 0.0;
    // Feedback (PID) constants
    public static final double kPsim = 0.0;
    public static final double kDsim = 0.0;
  }

  /** Climb Mechanism Constants ******************************************** */
  public static final class ClimbConstants {

    public static final double kClimbGearRatio = 1.0;
    // Mechanism idle mode
    public static final MotorIdleMode kClimbIdleMode = MotorIdleMode.COAST; // BRAKE, COAST

    // Flywheel motor open-loop and closed-loop ramp periods for current smoothing
    //   Time from from 0 -> full duty
    public static final double kClimbClosedLoopRampPeriod = 0.15; // seconds
    public static final double kClimbOpenLoopRampPeriod = 0.25; // seconds

    // magic configs
    //   public static final double mm_cruiseVelocity = 80;
    //   public static final double mm_acceleration = 160;
    //   public static final double mm_jerk = 1600;

    //   // magic PIDSVA
    //   public static final double mm_kP = 0.25;
    //   public static final double mm_kI = 0.0;
    //   public static final double mm_kD = 0.25;
    //   public static final double mm_kS = 0.25;
    //   public static final double mm_kV = 0.25;
    //   public static final double mm_kA = 0.25;

    public static final double kP = 6;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  /** Intake Mechanism Constants ******************************************* */
  public static final class IntakePivotConstants {

    // magic configs
    public static final double mm_cruiseVelocity = 80;
    public static final double mm_acceleration = 160;
    public static final double mm_jerk = 1600;

    // magic PIDSVA
    public static final double mm_kP = 0.25;
    public static final double mm_kI = 0.0;
    public static final double mm_kD = 0.25;
    public static final double mm_kS = 0.25;
    public static final double mm_kV = 0.25;
    public static final double mm_kA = 0.25;
  }

  /************************************************************************* */
  /** Turret Constants **************************************************** */
  public static final class TurretConstants {
    public static final double kTurretGearRatio = 10;
    public static final double kHoodAngle = 71.5;
    public static final int encoderID = 43;

    public static final double kP = 1.5;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  /************************************************************************* */

  /************************************************************************* */
  /** (Semi-)Autonomous Action Constants *********************************** */
  public static final class AutoConstants {

    // ********** PATHPLANNER CONSTANTS *******************
    // PathPlanner Config constants
    public static final RobotConfig kPathPlannerConfig =
        new RobotConfig(
            RobotConstants.kRobotMass.in(Kilograms),
            RobotConstants.kRobotMOI,
            new ModuleConfig(
                DrivebaseConstants.kWheelRadiusMeters,
                DrivebaseConstants.kMaxLinearSpeed,
                RobotConstants.kWheelCOF,
                DCMotor.getKrakenX60Foc(1).withReduction(SwerveConstants.kDriveGearRatio),
                DrivebaseConstants.kSlipCurrent,
                1),
            Drive.getModuleTranslations());

    // Alternatively, we can build this from the PathPlanner GUI:
    // public static final RobotConfig kPathPlannerConfig = RobotConfig.fromGUISettings();

    // ********** CHOREO CONSTANTS ************************
    // Drive and Turn PID constants used for ChoreO
    public static final PIDConstants kChoreoDrivePID = new PIDConstants(10.0, 0.0, 0.0);
    public static final PIDConstants kChoreoSteerPID = new PIDConstants(7.5, 0.0, 0.0);

    // ********** AUTOPILOT CONSTANTS *********************
    // Autopilot (Drive to Pose in Teleop) Constraints
    // see https://therekrab.github.io/autopilot/usage.html

    // Acceleration and Jerk to be applied
    private static final APConstraints kAPConstraints =
        new APConstraints().withAcceleration(3.0).withJerk(3.0);

    // Motion profile for drive to pose
    private static final APProfile kAPProfile =
        new APProfile(kAPConstraints)
            .withErrorXY(Centimeters.of(1))
            .withErrorTheta(Degrees.of(0.2))
            .withBeelineRadius(Centimeters.of(8));

    // Autopilot object to be used for specific commands
    public static final Autopilot kAutopilot = new Autopilot(kAPProfile);
  }

  /************************************************************************* */
  /** Vision Constants (Assuming PhotonVision) ***************************** */
  public static class VisionConstants {

    public static final Set<Integer> kTrustedTags =
        Set.of(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27); // HUB AprilTags

    // Noise scaling factors (lower = more trusted)
    public static final double kTrustedTagStdDevScale = 0.6; // 40% more weight
    public static final double kUntrustedTagStdDevScale = 1.3; // 30% less weight

    // Optional: if true, reject observations that contain no trusted tags
    public static final boolean kRequireTrustedTag = false;

    // AprilTag Identification Constants
    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    public static final double kZMargin = 0.75;
    public static final double kXYZStdDevCoefficient = 0.005;
    public static final double kThetaStdDevCoefficient = 0.01;

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  /************************************************************************* */
  /** Vision Camera Posses ************************************************* */
  public static final class Cameras {
    public record CameraConfig(
        String name,
        Transform3d robotToCamera,
        double stdDevFactor,
        SimCameraProperties simProps) {}

    // Camera Configuration Records
    // (ONLY USED FOR PHOTONVISION -- Limelight: configure in web UI instead)
    // Example Cameras are mounted in the back corners, 18" up from the floor, facing sideways
    public static final CameraConfig[] ALL = {
      new CameraConfig(
          "Photon_BW7",
          new Transform3d(
              Inches.of(-11.25),
              Inches.of(13.5),
              Inches.of(15.5),
              new Rotation3d(0.0, Units.degreesToRadians(10.), Math.PI / 2)),
          1.0,
          new SimCameraProperties() {
            {
              setCalibration(1280, 800, Rotation2d.fromDegrees(150));
              setCalibError(0.25, 0.08);
              setFPS(30);
              setAvgLatencyMs(20);
              setLatencyStdDevMs(5);
            }
          }),
      //
      new CameraConfig(
          "Photon_C10",
          new Transform3d(
              Inches.of(-11.25),
              Inches.of(-13.5),
              Inches.of(15.5),
              new Rotation3d(0.0, Units.degreesToRadians(10.), -Math.PI / 2)),
          1.0,
          new SimCameraProperties() {
            {
              setCalibration(1280, 800, Rotation2d.fromDegrees(120));
              setCalibError(0.25, 0.08);
              setFPS(30);
              setAvgLatencyMs(20);
              setLatencyStdDevMs(5);
            }
          }),
      new CameraConfig(
          "PC_Camera",
          new Transform3d(
              Inches.of(-13.5),
              Inches.of(0),
              Inches.of(12),
              new Rotation3d(0.0, Units.degreesToRadians(12.), -Math.PI)),
          0.75,
          new SimCameraProperties() {
            {
              setCalibration(1280, 800, Rotation2d.fromDegrees(120));
              setCalibError(0.25, 0.08);
              setFPS(30);
              setAvgLatencyMs(20);
              setLatencyStdDevMs(5);
            }
          }),

      // ... And more, if needed
    };
  }

  /************************************************************************* */
  /** Deploy Directoy Location Constants *********************************** */
  public static final class DeployConstants {
    public static final String apriltagDir = "apriltags";
    public static final String choreoDir = "choreo";
    public static final String pathplannerDir = "pathplanner";
    public static final String yagslDir = "swerve";
  }

  /***************************************************************************/
  /** Getter functions -- do not modify ************************************ */
  /** Get the current robot */
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert(
              "Invalid robot selected, using competition robot as default.", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  /** Get the current robot mode */
  public static Mode getMode() {
    return switch (robotType) {
      case GEORGE, PINCHY, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  /** Return whether this is pure simulation */
  public static boolean isPureSim() {
    boolean isReplay = Logger.hasReplaySource();
    return getMode() == Mode.SIM && !isReplay;
  }

  /** Get the current swerve drive type */
  public static SwerveType getSwerveType() {
    return swerveType;
  }

  /** Get the current autonomous path planning type */
  public static AutoType getAutoType() {
    return autoType;
  }

  /** Get the current autonomous path planning type */
  public static VisionType getVisionType() {
    return visionType;
  }

  /** Get the current CTRE/Phoenix Pro License state */
  public static CTREPro getPhoenixPro() {
    return phoenixPro;
  }

  /** Get the current AprilTag layout type. */
  public static AprilTagLayoutType getAprilTagLayoutType() {
    return FieldConstants.defaultAprilTagType;
  }
}
