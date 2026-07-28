package frc.robot.subsystems.extender;

import static frc.robot.Constants.ExtenderConstants.*;
import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.PowerConstants;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.RBSIEnum.CTREPro;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class ExtenderIOTalonFX implements ExtenderIO {
  private final TalonFX extenderLeader =
      new TalonFX(PIVOT_LEADER.getDeviceNumber(), PIVOT_LEADER.getCANBus());
  private final TalonFX extenderFollower =
      new TalonFX(PIVOT_FOLLOWER.getDeviceNumber(), PIVOT_FOLLOWER.getCANBus());
  public final int[] powerPorts = {PIVOT_LEADER.getPowerPort(), PIVOT_FOLLOWER.getPowerPort()};

  @Override
  public int[] powerPorts() {
    return powerPorts;
  }

  private final StatusSignal<Angle> leaderPosition = extenderLeader.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = extenderLeader.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = extenderLeader.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = extenderLeader.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = extenderFollower.getSupplyCurrent();

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  public ExtenderIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrentAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode =
        switch (kIdleMode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };

    OpenLoopRampsConfigs openRamps = new OpenLoopRampsConfigs();
    openRamps.DutyCycleOpenLoopRampPeriod = kOpenLoopRampPeriodSecs;
    openRamps.VoltageOpenLoopRampPeriod = kOpenLoopRampPeriodSecs;
    openRamps.TorqueOpenLoopRampPeriod = kOpenLoopRampPeriodSecs;
    ClosedLoopRampsConfigs closedRamps = new ClosedLoopRampsConfigs();
    closedRamps.DutyCycleClosedLoopRampPeriod = kClosedLoopRampPeriodSecs;
    closedRamps.VoltageClosedLoopRampPeriod = kClosedLoopRampPeriodSecs;
    closedRamps.TorqueClosedLoopRampPeriod = kClosedLoopRampPeriodSecs;

    config.withClosedLoopRamps(closedRamps).withOpenLoopRamps(openRamps);

    PhoenixUtil.tryUntilOk(5, () -> extenderLeader.getConfigurator().apply(config, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> extenderFollower.getConfigurator().apply(config, 0.25));
    extenderFollower.setControl(
        new Follower(extenderLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    extenderLeader.optimizeBusUtilization();
    extenderFollower.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / kGearRatio;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / kGearRatio;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
    inputs.extenderConnected = status.isOK();
  }

  @Override
  public void stop() {
    extenderLeader.stopMotor();
  }

  @Override
  public void setVoltage(double volts) {
    extenderLeader.setControl(voltageRequest.withOutput(volts).withEnableFOC(isCTREPro));
  }

  @Override
  public void setPercent(double percent) {
    extenderLeader.setControl(dutyCycleRequest.withOutput(percent).withEnableFOC(isCTREPro));
  }

  @Override
  public double getPos() {
    return Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / kGearRatio;
  }

  @Override
  public void setPivotVelocity(double velocity) {
    setPercent(velocity);
  }

  @Override
  public void goUntilPosition(double position) {
    extenderLeader.setControl(
        positionRequest
            .withPosition(Units.radiansToRotations(position) * kGearRatio)
            .withEnableFOC(isCTREPro));
  }

  @Override
  public double downPos() {
    return kDownPositionRad;
  }

  @Override
  public void configPID(double kP, double kI, double kD) {
    configureGains(kP, kI, kD, 0.0, 0.0, 0.0);
  }

  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV) {
    configureGains(kP, kI, kD, kS, kV, 0.0);
  }

  @Override
  public void configureGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    PhoenixUtil.tryUntilOk(5, () -> extenderLeader.getConfigurator().apply(config, 0.25));
  }
}
