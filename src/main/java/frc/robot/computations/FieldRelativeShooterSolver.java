// Copyright 2026 FRC 2486
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

package frc.robot.computations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;

public final class FieldRelativeShooterSolver {
  private FieldRelativeShooterSolver() {}

  public record FieldShotSolution(
      double v0,
      Rotation2d psiField,
      double time,
      FixedThetaBallistics.Branch branchAtEndpoint,
      double zApex,
      FixedThetaBallistics.CrossingDistances crossingsAtDz) {
    public double getVelocity() {
      return this.v0;
    }

    public Rotation2d getAngle() {
      return this.psiField;
    }
  }

  private static FixedThetaBallistics.Solution local;

  /**
   * @param fieldRobotPose robot/platform pose in FIELD frame
   * @param launcherTransformRobot transform from ROBOT origin to LAUNCHER exit (robot frame)
   * @param fieldTargetPose target pose in FIELD frame
   * @param fieldPlatformVelocityMps platform horizontal velocity in FIELD frame (m/s)
   */
  public static FieldShotSolution solve(
      Pose3d fieldRobotPose,
      Transform3d launcherTransformRobot,
      Pose3d fieldTargetPose,
      Translation2d fieldPlatformVelocityMps) {
    // Launcher pose in field frame
    Pose3d fieldLauncherPose = fieldRobotPose.plus(launcherTransformRobot);

    // Target expressed in launcher frame
    Transform3d targetInLauncherFrame = new Transform3d(fieldLauncherPose, fieldTargetPose);

    double dxL = targetInLauncherFrame.getX();
    double dyL = targetInLauncherFrame.getY();
    double dzL = targetInLauncherFrame.getZ();

    // Rotate platform velocity from field frame into launcher frame:
    // launcher frame is field frame rotated by launcher yaw, so apply inverse rotation.
    double yaw = fieldLauncherPose.getRotation().getZ();
    double c = Math.cos(-yaw);
    double s = Math.sin(-yaw);

    double vFx = fieldPlatformVelocityMps.getX();
    double vFy = fieldPlatformVelocityMps.getY();

    double vLx = c * vFx - s * vFy;
    double vLy = s * vFx + c * vFy;

    // Solve in launcher frame
    try {
      local = FixedThetaBallistics.solveDescendingWithApexClearance(dxL, dyL, dzL, vLx, vLy);
    } catch (IllegalArgumentException | IllegalStateException e) {
      // If exception raised in the solver, use a "blank" solution w/ v0 = 0.
      local = FixedThetaBallistics.blankSolution();
    }

    // Convert psi from launcher frame to field frame yaw
    double psiFieldRad = MathUtil.angleModulus(local.psiRad() + yaw);

    return new FieldShotSolution(
        local.v0(),
        Rotation2d.fromRadians(psiFieldRad),
        local.timeHit(),
        local.branchAtEndpoint(),
        local.zApex(),
        local.crossings());
  }
}
