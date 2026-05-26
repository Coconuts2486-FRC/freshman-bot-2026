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

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// public class ClimbIOSim implements ClimbIO {

//   private final DCMotor m_gearbox = DCMotor.getNEO(1);
//   private final LinearSystem<N1, N1, N1> m_plant =
//       LinearSystemId.createFlywheelSystem(m_gearbox, 1.0, 1.0);

//   private final FlywheelSim sim = new FlywheelSim(m_plant, m_gearbox);
//   private PIDController pid = new PIDController(0, 0, 0);

//   private boolean closedLoop = false;
//   private double ffVolts = 0.0;
//   private double appliedVolts = 0.0;

//   /** Constructor */
//   public ClimbIOSim() {}

//   @Override
//   public void updateInputs(ClimbIOInputs inputs) {
//     if (closedLoop) {
//       appliedVolts =
//           MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0,
// 12.0);
//       sim.setInputVoltage(appliedVolts);
//     }
//   }

//   /** Return the SIM power ports */
//   @Override
//   public int[] powerPorts() {
//     return new int[] {};
//   }
// }
