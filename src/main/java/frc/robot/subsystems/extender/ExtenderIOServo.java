package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj.Servo;

public class ExtenderIOServo implements ExtenderIO {
  final Servo servo = new Servo(9);

  @Override
  public void goToPos(double servoPos) {
    servo.set(servoPos);
  }

  @Override
  public double getPos() {
    return servo.getPosition();
  }
}
