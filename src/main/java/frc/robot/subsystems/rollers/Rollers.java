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

package frc.robot.subsystems.rollers;

import static frc.robot.Constants.IntakeConstants.*;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Rollers extends RBSISubsystem {
  private RollersIO io;
  private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

  public Rollers(RollersIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getMode()) {
      case REAL:
      case REPLAY:
        io.configureGains(kPreal, 0.0, kDreal, kSreal, kVreal, kAreal);
        break;
      case SIM:
      default:
        io.configureGains(kPsim, 0.0, kDsim, kSsim, kVsim, kAsim);
        break;
    }
  }

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
    Logger.recordOutput("Rollers/RollersRunning", (Math.abs(inputs.velocityRadPerSec) > 100));
  }

  /**
   * Run the intake rollers
   *
   * <p>For intake, the motor needs to run BACKWARDS!
   */
  public void runRollers() {
    io.setVelocity(-IntakeConstants.kRollersRPM / 60.);
    // io.runRollers(-0.8);
  }

  /** Run the intake rollers in reverse */
  public void reverseRollers() {
    io.setVelocity(IntakeConstants.kRollersRPM / 60.);
    // io.runRollers(0.8);
  }

  /** Run the intake rollers slowly while shooting */
  public void feedRollers() {
    io.setVelocity(-IntakeConstants.kRollersRPM * 0.8 / 60.);
    // io.runRollers(0.33);
  }

  public void stop() {
    io.stop();
  }

  public boolean isIntakeRollersRunning() {
    return io.isIntakeRollersRunning();
  }

  public boolean isRollersAlive() {
    return inputs.rollersConnected;
  }

  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
