package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public final class CompbotTunerView implements TunerView {
  @Override
  public CANBus canBus() {
    return CompbotTunerConstants.kCANBus;
  }

  @Override
  public SwerveDrivetrainConstants drivetrain() {
    return CompbotTunerConstants.DrivetrainConstants;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft() {
    return CompbotTunerConstants.FrontLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight() {
    return CompbotTunerConstants.FrontRight;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft() {
    return CompbotTunerConstants.BackLeft;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight() {
    return CompbotTunerConstants.BackRight;
  }
}
