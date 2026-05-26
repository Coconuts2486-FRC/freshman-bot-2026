package frc.robot.subsystems.feeder;

import static frc.robot.Constants.RobotDevices.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX feeder =
      new TalonFX(FEEDER_ROLLER.getDeviceNumber(), FEEDER_ROLLER.getCANBus());

  public final int[] POWER_PORTS = {FEEDER_ROLLER.getPowerPort()};

  private final StatusSignal<Angle> feederPosition = feeder.getPosition();
  private final StatusSignal<AngularVelocity> feederVelocity = feeder.getVelocity();
  private final StatusSignal<Voltage> feederAppliedVolts = feeder.getMotorVoltage();
  private final StatusSignal<Current> feederCurrent = feeder.getSupplyCurrent();

  // configs
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final boolean isCTREPro = Constants.getPhoenixPro() == CTREPro.LICENSED;

  /**
   * Constructor
   * *************************************************************************************************************
   */
  public FeederIOTalonFX() {
    config.CurrentLimits.SupplyCurrentLimit = PowerConstants.kMotorPortMaxCurrent;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Apply the configurations to the Shooter motors
    PhoenixUtil.tryUntilOk(5, () -> feeder.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, feederPosition, feederVelocity, feederAppliedVolts, feederCurrent);
    feeder.optimizeBusUtilization();
  }

  // Update Inputs
  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            feederPosition, feederVelocity, feederAppliedVolts, feederCurrent);
    inputs.feederAlive = status.isOK();
    inputs.positionRad = Units.rotationsToRadians(feederPosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(feederVelocity.getValueAsDouble());
    inputs.appliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {feederCurrent.getValueAsDouble()};
  }

  // Return the power ports
  @Override
  public int[] powerPorts() {
    return POWER_PORTS;
  }

  /**
   * Base Functions
   * *********************************************************************************************************
   */

  // controls motor with value between -1 and 1 0 being off and 1 being 100%
  @Override
  public void setFeederVelocity(double velocity) {
    feeder.set(velocity);
  }

  // Stop the feeder
  @Override
  public void stopFeeder() {
    feeder.stopMotor();
    feeder.setControl(new MotionMagicDutyCycle(0.));
  }

  /**
   * Getter Functions
   * ****************************************************************************************************
   */

  // returns true if feeder is going at above 10%
  @Override
  public boolean isFeederRunning() {
    return (Math.abs(feeder.get()) > 0.1);
  }
}
