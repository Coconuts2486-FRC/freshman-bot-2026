// Copyright (c) 2026 FRC-2486
// https://github.com/Coconuts2486-FRC
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

package frc.robot.subsystems.coordinator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.ShooterConstants.kShooterTransform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ShooterConstants;
import frc.robot.FieldConstants;
import frc.robot.computations.BasicRegression;
import frc.robot.computations.BasicRegression.RegressionShotSolution;
import frc.robot.computations.EpicRegression;
import frc.robot.computations.EpicRegression.EpicShotSolution;
import frc.robot.computations.FieldRelativeShooterSolver;
import frc.robot.computations.FieldRelativeShooterSolver.FieldShotSolution;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Coordinator extends VirtualSubsystem {
  public enum Mode {
    SYSTEM_CHECK, // Disabled, pre-match
    IDLE, // Disabled, playing defense, climb
    CLIMB // Climbing
  }

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Translation2d> velocitySupplier;
  private Mode mode = Mode.IDLE;

  // Instantiate loop variables
  private static Pose2d pose;
  private double xpos;
  private double ypos;
  private Translation2d velocity;
  private Alliance alliance = Alliance.Blue;
  private boolean allianceSet = false;
  boolean runOnceDisabled = true;

  // Internal variables
  private static boolean ok_to_shoot = false;
  public static Pose3d target = null;
  private static FieldShotSolution physicsSolution;
  private static RegressionShotSolution fuelSolution;
  private static EpicShotSolution epicSolution;
  private double midField = FieldConstants.aprilTagLayout.getFieldWidth() / 2.;

  private enum Zones {
    HOME_ZONE,
    NEUTRAL_ZONE,
    FOREIGN_ZONE
  }

  private Zones zone;

  /** Constructor */
  public Coordinator(Supplier<Pose2d> poseSupplier, Supplier<Translation2d> velocitySupplier) {
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
  }

  public void setMode(Mode mode) {
    this.mode = mode;
  }

  public static boolean isAimedAtTarget(double toleranceDegrees) {
    if (epicSolution == null || pose == null || target == null) return false;

    Rotation2d desired = epicSolution.getAngle();

    Rotation2d current = pose.getRotation();

    return Math.abs(current.minus(desired).getDegrees()) <= toleranceDegrees;
  }

  @Override
  public void rbsiPeriodic() {
    if (DriverStation.isDisabled() && !runOnceDisabled) {
      // Run the whole function once in disabled to init everything and make ENABLED startup faster
      runOnceDisabled = false;
      // Always safe outputs
      return;
    }

    // Read in the current robot state "truth"
    pose = poseSupplier.get();
    xpos = pose.getX();
    ypos = pose.getY();
    velocity = velocitySupplier.get();
    if (!allianceSet && DriverStation.isEnabled()) {
      // Get the current alliance once when enabled
      alliance = DriverStation.getAlliance().get();
      allianceSet = true;
    }

    // Determine whether we are in the HOME, NEUTRAL, or FOREIGN zone
    if (xpos < FieldConstants.startingLineXBlueMeters) {
      // On the BLUE side of the field
      zone =
          switch (alliance) {
            case Blue -> Zones.HOME_ZONE;
            case Red -> Zones.FOREIGN_ZONE;
          };

    } else if (xpos > FieldConstants.startingLineXRedMeters) {
      // On the RED side of the field
      zone =
          switch (alliance) {
            case Red -> Zones.HOME_ZONE;
            case Blue -> Zones.FOREIGN_ZONE;
          };
    } else {
      // In the NEUTRAL ZONE
      zone = Zones.NEUTRAL_ZONE;
    }

    // Choose shooting action based on zone
    switch (zone) {
      case HOME_ZONE:
        // Aim turret at our hub
        target =
            switch (alliance) {
              case Blue -> FieldConstants.hubCenterBlue;
              case Red -> FieldConstants.hubCenterRed;
            };

        break;

      case NEUTRAL_ZONE:
        // Aim turret at one of two passing locations based on Y position
        switch (alliance) {
          case Blue:
            target =
                (ypos < midField)
                    ? FieldConstants.passingOutpostBlue
                    : FieldConstants.passingDepotBlue;

          case Red:
            target =
                (ypos > midField)
                    ? FieldConstants.passingOutpostRed
                    : FieldConstants.passingDepotRed;
        }
        break;

      case FOREIGN_ZONE:
        // Do nothing!
        target = Pose3d.kZero;
        break;
    }

    // Using the target and the current pose, compute v0 and phi
    // fuelSolution =
    //     FieldRelativeShooterSolver.solve(new Pose3d(pose), kShooterTransform, target, velocity);
    fuelSolution = BasicRegression.solve(new Pose3d(pose), kShooterTransform, target);

    epicSolution = EpicRegression.solve(new Pose3d(pose), kShooterTransform, target, velocity);

    physicsSolution =
        FieldRelativeShooterSolver.solve(new Pose3d(pose), kShooterTransform, target, velocity);

    // 2) State machine / mode logic
    switch (mode) {
      case IDLE -> {
        // default behavior (maybe driver control only)
      }

      // case AIM -> {
      //   if (wantAutoAim && tgt.isPresent()) {
      //     // Example: compute desired heading from target solution
      //     double desiredHeadingRad = targeting.getDesiredRobotHeadingRad(pose, tgt.get());

      //     // Produce a chassis request (you might have your own helper)
      //     ChassisSpeeds speeds = targeting.buildAimingDriveRequest(desiredHeadingRad);
      //     drive.runVelocity(speeds);
      //   }
      // }

      case CLIMB -> {
        // climb logic
      }

      default -> {}
    }

    // 3) Log coordinator outputs for tuning
    // Logger.recordOutput("Coord/Mode", mode.toString());
    // Logger.recordOutput("Coord/WantAutoAim", wantAutoAim);

    // But really do this based on calculating things!
    // Based on field position, HUB active, turret in position, flywheel at speed, override not set,
    // "DON'T SHOOT" button not pressed...

    // double dist2hub = pose.minus(pose);

    Logger.recordOutput(
        "Coordinator/RegVel", RotationsPerSecond.of(fuelSolution.getVelocity() / 60.));
    Logger.recordOutput(
        "Coordinator/PhysVel",
        RotationsPerSecond.of(
            physicsSolution.getVelocity() / ShooterConstants.kFlywheelCircumfrence));
    Logger.recordOutput(
        "Coordinator/EpicVel",
        RotationsPerSecond.of(epicSolution.getVelocity() / ShooterConstants.kFlywheelCircumfrence));
  }

  // Getter functions
  public static boolean getOk2Shoot() {
    return ok_to_shoot;
  }

  public static double getShooterVelocity() {
    return fuelSolution.getVelocity();
  }

  public static Rotation2d getTurretAngle() {
    return fuelSolution.getAngle();
  }

  public static double getEpicShooterVelocity() {
    return epicSolution.getVelocity();
  }

  public static Rotation2d getRobotAngle() {
    return epicSolution.getAngle();
  }
}
