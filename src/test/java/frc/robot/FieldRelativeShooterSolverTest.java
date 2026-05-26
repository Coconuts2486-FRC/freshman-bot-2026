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

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import org.junit.jupiter.api.Test;

public class FieldRelativeShooterSolverTest {

  @Test
  void yawIsFieldRelative() {
    Pose3d robot = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, Math.PI / 2.0));

    Transform3d launcher = new Transform3d(new Translation3d(0.5, 0.0, 1.0), new Rotation3d());

    Pose3d target = new Pose3d(new Translation3d(0.0, 5.5, 2.32), new Rotation3d());

    var sol = FieldRelativeShooterSolver.solve(robot, launcher, target, new Translation2d());

    assertEquals(Math.PI / 2.0, sol.psiField().getRadians(), 1e-2);
  }

  @Test
  void platformVelocityTowardTargetReducesV0() {
    Pose3d robot = new Pose3d();
    Transform3d launcher = new Transform3d(new Translation3d(0.5, 0.0, 1.0), new Rotation3d());

    Pose3d target = new Pose3d(new Translation3d(5.5, 0.0, 2.32), new Rotation3d());

    var stationary =
        FieldRelativeShooterSolver.solve(robot, launcher, target, new Translation2d(0, 0));

    var toward =
        FieldRelativeShooterSolver.solve(robot, launcher, target, new Translation2d(1.0, 0.0));

    assertTrue(toward.v0() < stationary.v0());
  }
}
