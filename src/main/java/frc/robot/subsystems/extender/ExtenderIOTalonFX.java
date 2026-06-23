package frc.robot.subsystems.extender;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ExtenderIOTalonFX implements ExtenderIO {
  private final TalonFX extenderMotor =
      new TalonFX(EXTENDER_MOTOR.getDeviceNumber(), EXTENDER_MOTOR.getCANBus());
  private final CANcoder extenderEncoder =
      new CANcoder(EXTENDER_ENCODER.getDeviceNumber(), EXTENDER_ENCODER.getCANBus());


      
  private final StatusSignal<Angle> extenderPosition = null;
  private final StatusSignal<AngularVelocity> extenderVelocity = null;
  private final StatusSignal<Voltage> extenderAppliedVolts = null;
  private final StatusSignal<Current> extenderCurrent = null;
  
  private final PIDController pid = new PIDController(0,0,0);

  @Override
  public void stop() {
    extenderMotor.stopMotor();
  }

@Override
  public void setPivotVelocity(double velocity){
  extenderMotor.set(velocity);
  }
@Override
  public void goUntilPosition(double position){
  extenderMotor.set(pid.calculate(extenderEncoder.getAbsolutePosition().getValueAsDouble(),position));
  }

@Override
  public double downPos() {
  return 0.0;
  } 

@Override 
  public void configPID(double kP, double kI, double kD) {}

}