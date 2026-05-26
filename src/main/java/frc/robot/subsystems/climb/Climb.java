// // Copyright (c) 2026 FRC-2486
// // https://github.com/Coconuts2486-FRC
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.
// //
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.climb;

// import frc.robot.util.RBSISubsystem;
// import org.littletonrobotics.junction.Logger;

// public class Climb extends RBSISubsystem {

//   // Declare IO
//   private ClimbIO io;
//   private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

//   /** Constructor */
//   public Climb(ClimbIO io) {
//     this.io = io;
//   }

//   // Periodic function- runs every 20ms (~50x/second)
//   @Override
//   public void rbsiPeriodic() {
//     io.updateInputs(inputs);
//     Logger.processInputs("Climb", inputs);
//   }

//   // Sets position using PID based off of desired position
//   public void setPosition(double pos) {
//     io.setPosition(pos);
//   }

//   // Checks if the climb is connected to the CAN network
//   public boolean climbAlive() {
//     return inputs.climbAlive;
//   }

//   public double climbPosition() {
//     return io.getPosition();
//   }

//   @Override
//   public int[] getPowerPorts() {
//     return io.getPowerPorts();
//   }
// }
