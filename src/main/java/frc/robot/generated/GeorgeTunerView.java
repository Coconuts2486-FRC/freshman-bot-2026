package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public final class GeorgeTunerView implements TunerView {
  @Override
  public CANBus canBus() {
    return GeorgeTunerConstants.kCANBus;
  }

  @Override
  public SwerveDrivetrainConstants drivetrain() {
    return GeorgeTunerConstants.DrivetrainConstants;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft() {
    return GeorgeTunerConstants.FrontLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight() {
    return GeorgeTunerConstants.FrontRight;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft() {
    return GeorgeTunerConstants.BackLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight() {
    return GeorgeTunerConstants.BackRight;
  }
}
