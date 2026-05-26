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
import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RBSIEnum.CTREPro;

public class RollersIOTalonFX implements RollersIO {

  private final TalonFX rollers =
      new TalonFX(INTAKE_ROLLER.getDeviceNumber(), INTAKE_ROLLER.getCANBus());

  private final StatusSignal<Angle> rollersPosition = rollers.getPosition();
  private final StatusSignal<AngularVelocity> rollersVelocity = rollers.getVelocity();
  private final StatusSignal<Voltage> rollersAppliedVolts = rollers.getMotorVoltage();
  private final StatusSignal<Current> rollersCurrent = rollers.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(0.0).withSlot(0);

  /** Constructor */
  public RollersIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kIntakeIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };
    // Build the OpenLoopRampsConfigs and ClosedLoopRampsConfigs for current smoothing
    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kShooterOpenLoopRampPeriod;
    openRamps.VoltageOpenLoopRampPeriod = kShooterOpenLoopRampPeriod;
    openRamps.TorqueOpenLoopRampPeriod = kShooterOpenLoopRampPeriod;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kShooterClosedLoopRampPeriod;
    closedRamps.VoltageClosedLoopRampPeriod = kShooterClosedLoopRampPeriod;
    closedRamps.TorqueClosedLoopRampPeriod = kShooterClosedLoopRampPeriod;
    // Apply the open- and closed-loop ramp configuration for current smoothing
    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);

    // Apply the configurations to the Shooter motors
    PhoenixUtil.tryUntilOk(5, () -> rollers.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollersPosition, rollersVelocity, rollersAppliedVolts, rollersCurrent);
    rollers.optimizeBusUtilization();
  }

  /** Update inputs */
  @Override
  public void updateInputs(RollersIOInputs inputs) {

    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollersPosition, rollersVelocity, rollersAppliedVolts, rollersCurrent);

    inputs.rollersConnected = rollerStatus.isOK();
    inputs.positionRad =
        Units.rotationsToRadians(rollersPosition.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(rollersVelocity.getValueAsDouble()) / 1.0; // kShooterGearRatio;
    inputs.appliedVolts = rollersAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {rollersCurrent.getValueAsDouble()};
  }

  @Override
  /**
   * Set the velocity of the motor in rotations per second
   *
   * @param velocityRotationsPerSecond Desired angular speed in rotations per second
   */
  public void setVelocity(double velocityRotationsPerSecond) {
    rollers.setControl(velocityRequest.withVelocity(velocityRotationsPerSecond));
  }

  // @Override
  // public void runRollers(double speed) {
  //   rollers.set(speed);
  // }

  @Override
  public void stop() {
    rollers.stopMotor();
  }

  @Override
  public boolean isIntakeRollersRunning() {
    if (Math.abs(rollers.get()) > 0.02) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Set the gains of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   * @param kS Static gain
   * @param kV Velocity gain
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> rollers.getConfigurator().apply(config, 0.25));
  }

  /**
   * Set the gains of the Slot0 closed-loop configuration
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Differential gain
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   */
  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> rollers.getConfigurator().apply(config, 0.25));
  }
}
