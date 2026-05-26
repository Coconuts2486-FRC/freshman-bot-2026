// Copyright (c) 2024-2026 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2025-2026 Littleton Robotics
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

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  /** Specify which type of field your robot is using */
  private static final AprilTagLayoutType fieldType = AprilTagLayoutType.REBUILT_WELDED;

  public static final Distance startingLineXBLue = Inches.of(157.06);
  public static final double startingLineXBlueMeters = startingLineXBLue.in(Meters);

  public static final Distance startingLineXRed = Inches.of(493.06);
  public static final double startingLineXRedMeters = startingLineXRed.in(Meters);

  public static final Translation2d towerCenterBlue =
      new Translation2d(Inches.of(43.8), Inches.of(169.78));

  public static final Translation2d towerCenterRed =
      new Translation2d(Inches.of(607.42), Inches.of(147.47));

  public static final Pose3d hubCenterBlue =
      new Pose3d(
          new Translation3d(Inches.of(182.11), Inches.of(159.5), Inches.of(72)), Rotation3d.kZero);

  public static final Pose3d hubCenterRed =
      new Pose3d(
          new Translation3d(Inches.of(469.11), Inches.of(159.5), Inches.of(72)), Rotation3d.kZero);
  // 11.915394m and 4.034536m
  public static final Pose3d passingOutpostBlue =
      new Pose3d(
          new Translation3d(Inches.of(108.28), Inches.of(84.89), Inches.of(0)), Rotation3d.kZero);

  public static final Pose3d passingDepotBlue =
      new Pose3d(
          new Translation3d(Inches.of(108.28), Inches.of(267.85), Inches.of(0)), Rotation3d.kZero);

  public static final Pose3d passingOutpostRed =
      new Pose3d(
          new Translation3d(Inches.of(543.95), Inches.of(267.85), Inches.of(0)), Rotation3d.kZero);

  public static final Pose3d passingDepotRed =
      new Pose3d(
          new Translation3d(Inches.of(543.95), Inches.of(84.89), Inches.of(0)), Rotation3d.kZero);

  public static Translation2d hubCenter2d() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return new Translation2d(hubCenterRed.getX(), hubCenterRed.getY());
    } else if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return new Translation2d(hubCenterBlue.getX(), hubCenterBlue.getY());
    } else {
      return new Translation2d(hubCenterRed.getX(), hubCenterRed.getY());
    }
  }

  public static Translation2d shootingSpot() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return new Translation2d(hubCenterRed.getX() + 134, hubCenterRed.getY());
    } else {
      return new Translation2d(hubCenterBlue.getX() - 134, hubCenterBlue.getY());
    }
  }

  // public static Rotation2d targetAngle(Translation2d target, Pose2d robotPose) {
  //   if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
  //     return target.minus(robotPose.getTranslation()).getAngle();
  //   } else if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
  //     return target.minus(robotPose.getTranslation()).getAngle();
  //   } else {
  //     return target.minus(robotPose.getTranslation()).getAngle();
  //   }
  // }

  /** AprilTag Field Layout ************************************************ */
  public static final double aprilTagWidth = Inches.of(6.50).in(Meters);

  public static final String aprilTagFamily = "36h11";

  public static final double fieldLength = fieldType.getLayout().getFieldLength();
  public static final double fieldWidth = fieldType.getLayout().getFieldWidth();

  public static final int aprilTagCount = fieldType.getLayout().getTags().size();
  public static final AprilTagLayoutType defaultAprilTagType = fieldType;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  @Getter
  public enum AprilTagLayoutType {
    REBUILT_WELDED("2026-rebuilt-welded"),
    REBUILT_ANDYMARK("2026-rebuilt-welded"),
    REEFSCAPE_WELDED("2025-reefscape-welded"),
    REEFSCAPE_ANDYMARK("2025-reefscape-andymark"),
    CRESCENDO("2024-crescendo"),
    NONE_WELDED("none-welded"),
    NONE_ANDYMARK("none-andymark");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Constants.disableHAL
                    ? Path.of("src", "main", "deploy", "apriltags", name + ".json")
                    : Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      try {
        layoutString = new ObjectMapper().writeValueAsString(layout);
      } catch (JsonProcessingException e) {
        throw new RuntimeException(
            "Failed to serialize AprilTag layout JSON " + toString() + "for PhotonVision");
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
