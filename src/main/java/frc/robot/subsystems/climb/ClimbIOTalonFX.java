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

// import static frc.robot.Constants.ClimbConstants.*;
// import static frc.robot.Constants.RobotDevices.*;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
// import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.Constants;
// import frc.robot.Constants.ClimbConstants;
// import frc.robot.Constants.PowerConstants;
// import frc.robot.util.PhoenixUtil;
// import frc.robot.util.RBSIEnum.CTREPro;

// public class ClimbIOTalonFX implements ClimbIO {

//   // Declare Hardware
//   private final TalonFX climb_motor =
//       new TalonFX(CLIMB_MOTOR.getDeviceNumber(), CLIMB_MOTOR.getCANBus());
//   private final CANcoder climbEncoder =
//       new CANcoder(CLIMB_ENCODER.getDeviceNumber(), CLIMB_ENCODER.getCANBus());
//   public final int[] POWER_PORTS = {CLIMB_MOTOR.getPowerPort()};
//   PIDController ClimbPID =
//       new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);

//   // Define status signals
//   private final StatusSignal<Angle> climbPosition = climb_motor.getPosition();
//   private final StatusSignal<AngularVelocity> climbVelocity = climb_motor.getVelocity();
//   private final StatusSignal<Voltage> climbAppliedVolts = climb_motor.getMotorVoltage();
//   private final StatusSignal<Current> climbCurrent = climb_motor.getSupplyCurrent();

//   private final TalonFXConfiguration config = new TalonFXConfiguration();
//   private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

//   /** Return the power ports */
//   @Override
//   public int[] powerPorts() {
//     return POWER_PORTS;
//   }

//   /** Constructor */
//   public ClimbIOTalonFX() {
//     // Motion Magic Configs

//     // Current-limiting section
//     config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
//     config.CurrentLimits.SupplyCurrentLimitEnable = true;
//     config.MotorOutput.NeutralMode =
//         switch (kClimbIdleMode) {
//           case COAST -> NeutralModeValue.Coast;
//           case BRAKE -> NeutralModeValue.Brake;
//         };
//     // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
//     OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
//     openRamps.DutyCycleOpenLoopRampPeriod = kClimbOpenLoopRampPeriod;
//     openRamps.VoltageOpenLoopRampPeriod = kClimbOpenLoopRampPeriod;
//     openRamps.TorqueOpenLoopRampPeriod = kClimbOpenLoopRampPeriod;
//     ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
//     closedRamps.DutyCycleClosedLoopRampPeriod = kClimbClosedLoopRampPeriod;
//     closedRamps.VoltageClosedLoopRampPeriod = kClimbClosedLoopRampPeriod;
//     closedRamps.TorqueClosedLoopRampPeriod = kClimbClosedLoopRampPeriod;
//     // Apply the open- and closed-loop ramp configuration for current smoothing
//     config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);
//     // Apply to motor
//     PhoenixUtil.tryUntilOk(5, () -> climb_motor.getConfigurator().apply(config));

//     CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
//     cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
//     PhoenixUtil.tryUntilOk(5, () -> climbEncoder.getConfigurator().apply(cancoderConfig));
//   }

//   // Passes inputs for logging to the logger
//   @Override
//   public void updateInputs(ClimbIOInputs inputs) {
//     var climbStatus =
//         BaseStatusSignal.refreshAll(climbPosition, climbVelocity, climbAppliedVolts,
// climbCurrent);
//     inputs.climbAlive = climbStatus.isOK();
//     inputs.positionRad =
//         Units.rotationsToRadians(climbPosition.getValueAsDouble()) / kClimbGearRatio;
//     inputs.velocityRadPerSec =
//         Units.rotationsToRadians(climbVelocity.getValueAsDouble()) / kClimbGearRatio;
//     inputs.appliedVolts = climbAppliedVolts.getValueAsDouble();
//     inputs.currentAmps = new double[] {climbCurrent.getValueAsDouble()};
//   }

//   // Sets position with PID, calculating off of wanted position vs. current position
//   @Override
//   public void setPosition(double pos) {
//     // Can use TorqueFOC if isCTREPro is true
//     climb_motor.set(ClimbPID.calculate(pos,
// climbEncoder.getAbsolutePosition().getValueAsDouble()));
//   }

//   // Returns position of climb encoder as double
//   @Override
//   public double getPosition() {
//     return climbEncoder.getAbsolutePosition().getValueAsDouble();
//   }
// }
