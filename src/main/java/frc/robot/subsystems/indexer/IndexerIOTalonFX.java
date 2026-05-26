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

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

public class IndexerIOTalonFX implements IndexerIO {

  // Declare Hardware
  private final TalonFX indexer =
      new TalonFX(INDEXER_ROLLER.getDeviceNumber(), INDEXER_ROLLER.getCANBus());

  public final int[] POWER_PORTS = {INDEXER_ROLLER.getPowerPort()};

  private final StatusSignal<Angle> indexerPosition = indexer.getPosition();
  private final StatusSignal<AngularVelocity> indexerVelocity = indexer.getVelocity();
  private final StatusSignal<Voltage> indexerAppliedVolts = indexer.getMotorVoltage();
  private final StatusSignal<Current> indexerCurrent = indexer.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  /**
   * Constructor
   * ************************************************************************************************************
   */
  public IndexerIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply the configurations to the Shooter motors
    PhoenixUtil.tryUntilOk(5, () -> indexer.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, indexerPosition, indexerVelocity, indexerAppliedVolts, indexerCurrent);
    indexer.optimizeBusUtilization();
  }

  /**
   * Update inputs
   * ********************************************************************************************************
   */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var indexerStatus =
        BaseStatusSignal.refreshAll(
            indexerPosition, indexerVelocity, indexerAppliedVolts, indexerCurrent);

    inputs.indexerAlive = indexerStatus.isOK();
    inputs.positionRad = Units.rotationsToRadians(indexerPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(indexerVelocity.getValueAsDouble());
    inputs.appliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {indexerCurrent.getValueAsDouble()};
  }

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  // ** base functions
  // *********************************************************************************************************** */

  // sets velocity at value from -1 to 1 0 being off and 1 being max speed
  @Override
  public void setVelocity(double velocity) {
    indexer.set(velocity);
  }

  // stops indexer
  @Override
  public void indexerStop() {
    indexer.stopMotor();
  }
}
