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

import frc.robot.Constants.ShooterConstants;

public final class FixedThetaBallistics {
  private FixedThetaBallistics() {}

  public enum Branch {
    ASCENDING,
    DESCENDING
  }

  public record CrossingDistances(double rUp, double rApex, double rDown) {}

  public record Solution(
      double v0,
      double psiRad, // launcher-frame yaw
      double timeHit,
      Branch branchAtEndpoint,
      double zApex,
      CrossingDistances crossings) {}

  /** Descending-root time-of-flight to reach dz, given v0 and fixed theta. */
  private static double timeToDzDescending(double v0, double dz) {
    double g = ShooterConstants.kG;
    double theta = ShooterConstants.kThetaRad;

    double sinT = Math.sin(theta);
    double disc = v0 * v0 * sinT * sinT - 2.0 * g * dz;
    if (disc < 0.0) {
      throw new IllegalArgumentException("dz above reachable apex for this v0/theta");
    }
    return (v0 * sinT + Math.sqrt(disc)) / g; // descending root
  }

  /** Apex height above launch for given v0 and fixed theta. */
  private static double zApex(double v0) {
    double g = ShooterConstants.kG;
    double theta = ShooterConstants.kThetaRad;
    double sinT = Math.sin(theta);
    return (v0 * v0 * sinT * sinT) / (2.0 * g);
  }

  /** v_z at time t */
  private static double vz(double v0, double t) {
    double g = ShooterConstants.kG;
    double theta = ShooterConstants.kThetaRad;
    return v0 * Math.sin(theta) - g * t;
  }

  /**
   * Solve for v0 and psi in the launcher frame.
   *
   * <p>Inputs are launcher-frame deltas and launcher-frame platform horizontal velocity V=(vX,vY).
   *
   * <p>Enforces: - endpoint is DESCENDING (vz<0 at impact) - apex >= dz + h_c
   */
  public static Solution solveDescendingWithApexClearance(
      double dx, double dy, double dz, double vPlatX, double vPlatY) {
    double theta = ShooterConstants.kThetaRad;
    double g = ShooterConstants.kG;
    double hC = ShooterConstants.kApexClearanceMeters;

    double R = Math.hypot(dx, dy);
    if (R <= 0.0) throw new IllegalArgumentException("R must be > 0");

    // Minimum v0 to reach apex at least dz + hC
    double v0MinApex = Math.sqrt(2.0 * g * (dz + hC)) / Math.sin(theta);

    // Also must at least reach dz (usually weaker than apex constraint)
    double v0MinReach = Math.sqrt(Math.max(0.0, 2.0 * g * dz)) / Math.sin(theta);

    double v0Low = Math.max(Math.max(v0MinApex, v0MinReach), ShooterConstants.kMinBracket);

    // Define the 1D residual:
    // f(v0) = ||D/t(v0) - V|| - v0*cos(theta)
    // Root => horizontal consistency with platform velocity.
    java.util.function.DoubleUnaryOperator f =
        (v0) -> {
          double t = timeToDzDescending(v0, dz);
          double vxReq = dx / t - vPlatX;
          double vyReq = dy / t - vPlatY;
          double lhs = Math.hypot(vxReq, vyReq);
          double rhs = v0 * Math.cos(theta);
          return lhs - rhs;
        };

    // Bracket the root
    double fLow = f.applyAsDouble(v0Low);
    double v0High = v0Low;
    double fHigh = fLow;

    // Expand high until sign changes or we hit cap
    for (int i = 0; i < 60 && v0High < ShooterConstants.kMaxV0Search; i++) {
      v0High = Math.min(ShooterConstants.kMaxV0Search, v0High * 1.4 + 0.5);
      fHigh = f.applyAsDouble(v0High);
      if (fLow == 0.0 || fLow * fHigh <= 0.0) break;
    }

    if (!(fLow == 0.0 || fLow * fHigh <= 0.0)) {
      throw new IllegalArgumentException(
          "Could not bracket a valid v0 solution (unreachable under constraints).");
    }

    // Bisection
    double a = v0Low, b = v0High;
    double fa = fLow;
    double v0 = a;

    for (int iter = 0; iter < ShooterConstants.kMaxBisectionIters; iter++) {
      v0 = 0.5 * (a + b);
      double fm = f.applyAsDouble(v0);

      if (Math.abs(fm) < ShooterConstants.kV0Tol || (b - a) < ShooterConstants.kV0Tol) {
        break;
      }

      if (fa * fm <= 0.0) {
        b = v0;
      } else {
        a = v0;
        fa = fm;
      }
    }

    // Compute impact time, check descending explicitly
    double tHit = timeToDzDescending(v0, dz);
    double vzEnd = vz(v0, tHit);
    Branch branch = (vzEnd < 0.0) ? Branch.DESCENDING : Branch.ASCENDING;
    if (branch != Branch.DESCENDING) {
      throw new IllegalStateException("Internal error: expected descending impact, got ascending.");
    }

    // Apex clearance check explicitly
    double zA = zApex(v0);
    if (zA < dz + hC) {
      throw new IllegalArgumentException("Apex clearance violated after solve.");
    }

    // Solve psi (launcher frame): v0_h = D/t - V
    double v0hx = dx / tHit - vPlatX;
    double v0hy = dy / tHit - vPlatY;
    double psi = Math.atan2(v0hy, v0hx);

    // Useful "zone" distances: crossings at dz on up/apex/down.
    // Horizontal ground track in launcher frame is linear in time with speed |D|/tHit.
    double tApex = v0 * Math.sin(theta) / g;
    double disc = v0 * v0 * Math.sin(theta) * Math.sin(theta) - 2.0 * g * dz;
    double tUp = (v0 * Math.sin(theta) - Math.sqrt(disc)) / g; // ascending root

    double rUp = R * (tUp / tHit);
    double rApex = R * (tApex / tHit);
    double rDown = R; // endpoint by definition

    // Explicit ascending/descending zone ordering sanity check
    if (!(rUp < rApex && rApex < rDown)) {
      throw new IllegalStateException("Zone ordering violated: expected rUp < rApex < rDown.");
    }

    return new Solution(v0, psi, tHit, branch, zA, new CrossingDistances(rUp, rApex, rDown));
  }

  /**
   * Return a blank solution object in the event that
   *
   * @return
   */
  public static Solution blankSolution() {
    return new Solution(0.0, 0.0, 0.0, Branch.DESCENDING, 0.0, new CrossingDistances(0, 0, 0));
  }
}
