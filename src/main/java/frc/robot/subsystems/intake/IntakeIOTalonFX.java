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

package frc.robot.subsystems.intake;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {

  // Declare Hardware
  private final TalonFX pivot =
      new TalonFX(INTAKE_PIVOT.getDeviceNumber(), INTAKE_PIVOT.getCANBus());

  public final int[] POWER_PORTS = {INTAKE_PIVOT.getPowerPort(), INTAKE_ROLLER.getPowerPort()};

  private final CANcoder pivotEncoder =
      new CANcoder(INTAKE_ENCODER.getDeviceNumber(), INTAKE_ENCODER.getCANBus());

  private final StatusSignal<Angle> pivotPosition = pivot.getPosition();
  private final StatusSignal<AngularVelocity> pivotVelocity = pivot.getVelocity();
  private final StatusSignal<Voltage> pivotAppliedVolts = pivot.getMotorVoltage();
  private final StatusSignal<Current> pivotCurrent = pivot.getSupplyCurrent();

  /** Constructor */
  public IntakeIOTalonFX() {

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    // pivot
    pivotConfig.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    setCoast();

    pivotConfig.Slot0 =
        new Slot0Configs()
            .withKP(DrivebaseConstants.kSteerP)
            .withKI(0.0)
            .withKD(DrivebaseConstants.kSteerD)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // cancoder
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    cancoderConfig.MagnetSensor.MagnetOffset = 0.8;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, pivotCurrent, pivotPosition, pivotVelocity, pivotAppliedVolts);

    // applying
    PhoenixUtil.tryUntilOk(5, () -> pivotEncoder.getConfigurator().apply(cancoderConfig));
    pivot.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // checks the status of pivot
    var pivotStatus =
        BaseStatusSignal.refreshAll(pivotCurrent, pivotPosition, pivotVelocity, pivotAppliedVolts);

    // checks the status of roller
    inputs.pivotConnected = pivotStatus.isOK();
    inputs.pivotPositionRot = pivotPosition.getValueAsDouble();
    inputs.pivotAvAngularVelocity = pivotVelocity.getValueAsDouble();
    inputs.pivotAppliedVolts = pivotAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {pivotCurrent.getValueAsDouble()};
  }

  /** Return the power ports */
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /** Set the coast mode of the mechanism as COAST */
  @Override
  public void setCoast() {
    pivot.setNeutralMode(NeutralModeValue.Coast);
  }

  /** Set the coast mode of the mechanism as BRAKE */
  @Override
  public void setBrake() {
    pivot.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setPivotPrimitiveSpeed(double speed) {
    pivot.set(speed);
  }

  @Override
  public void stopPivot() {
    pivot.stopMotor();
  }

  /** Getter functions ***************************************************** */
  @Override
  public boolean isIntakeExtended() {
    return (pivotEncoder.getAbsolutePosition().getValueAsDouble()
        > (IntakeConstants.dropPosition - 0.05));
  }

  @Override
  public double getPivotPosition() {
    return pivotEncoder.getAbsolutePosition().getValueAsDouble();
  }
}
