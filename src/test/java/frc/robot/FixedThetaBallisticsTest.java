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

import org.junit.jupiter.api.Test;

public class FixedThetaBallisticsTest {

  // @Test
  // void v0ForR5_theta70_dz132_isCorrect() {
  //   // dx=5, dy=0, dz=1.32, platform stationary
  //   var sol = FixedThetaBallistics.solveDescendingWithApexClearance(5.0, 0.0, 1.32, 0.0, 0.0);

  //   assertEquals(9.19, sol.v0(), 0.05);
  //   assertEquals(FixedThetaBallistics.Branch.DESCENDING, sol.branchAtEndpoint());
  // }

  @Test
  void ascendingCrossingBeforeApexBeforeDescending() {
    var sol = FixedThetaBallistics.solveDescendingWithApexClearance(5.0, 0.0, 1.32, 0.0, 0.0);

    assertTrue(sol.crossings().rUp() < sol.crossings().rApex());
    assertTrue(sol.crossings().rApex() < sol.crossings().rDown());
  }

  @Test
  void apexClearanceEnforced() {
    // Make dz large enough to violate apex clearance
    assertThrows(
        IllegalArgumentException.class,
        () -> FixedThetaBallistics.solveDescendingWithApexClearance(2.0, 0.0, 2.0, 0.0, 0.0));
  }

  @Test
  void bisectionMonotonicitySanity() {
    // Tests that the solver is: correctly bracketed, monotonic in the physical regime, and not
    // accidentally converging to a wrong branch
    var sol1 = FixedThetaBallistics.solveDescendingWithApexClearance(5.0, 0.0, 1.32, 0.0, 0.0);
    var sol2 = FixedThetaBallistics.solveDescendingWithApexClearance(6.0, 0.0, 1.32, 0.0, 0.0);
    assertTrue(sol2.v0() > sol1.v0(), "Required v0 should increase monotonically with range");
  }
}
