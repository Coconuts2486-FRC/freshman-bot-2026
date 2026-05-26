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

package frc.robot.subsystems.indexer;

import frc.robot.util.RBSISubsystem;
import org.littletonrobotics.junction.Logger;

public class Indexer extends RBSISubsystem {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  // ** periodic functions
  // ************************************************************************************************** */

  @Override
  public void rbsiPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/stalling", (inputs.velocityRadPerSec > -40));
  }

  @Override
  public void simulationPeriodic() {}

  // ** base functions
  // ******************************************************************************************************* */

  // sets velocity at value from -1 to 1 0 being off and 1 being max speed
  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  // stops indexer motor
  public void indexerStop() {
    io.indexerStop();
  }

  // ** getter functions
  // **************************************************************************************************** */
  /** Get the velocity of the indexer */
  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  // returns boolean checking whether the motor is being recieved on CAN
  public boolean indexerAlive() {
    return inputs.indexerAlive;
  }

  // * power port fucntion */
  @Override
  public int[] getPowerPorts() {
    return io.getPowerPorts();
  }
}
