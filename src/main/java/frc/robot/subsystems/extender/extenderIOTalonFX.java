package frc.robot.subsystems.extender;

import static frc.robot.Constants.RobotDevices.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class extenderIOTalonFX implements extenderIO {
  private final TalonFX extenderMotor =
      new TalonFX(EXTENDER_MOTOR.getDeviceNumber(), EXTENDER_MOTOR.getCANBus());

  @Override
  public void stop() {
    extenderMotor.stopMotor();
  }
}
