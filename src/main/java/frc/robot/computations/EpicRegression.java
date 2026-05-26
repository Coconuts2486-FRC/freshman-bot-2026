package frc.robot.computations;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class EpicRegression {

  private static Pose2d shooter2d = Pose2d.kZero;
  private static Pose2d hub2d = Pose2d.kZero;
  private static Pose3d fieldLauncherPose;
  private static Translation2d translation;
  private static double v0;

  private EpicRegression() {}

  /** Regression Shot Solution Record */
  public record EpicShotSolution(double v0, Rotation2d psiField) {
    public double getVelocity() {
      return this.v0;
    }

    public Rotation2d getAngle() {
      return this.psiField;
    }
  }

  /**
   * @param fieldRobotPose robot/platform pose in FIELD frame
   * @param launcherTransformRobot transform from ROBOT origin to LAUNCHER exit (robot frame)
   * @param fieldTargetPose target pose in FIELD frame
   * @param fieldPlatformVelocityMps platform horizontal velocity in FIELD frame (m/s)
   */
  public static EpicShotSolution solve(
      Pose3d fieldRobotPose,
      Transform3d launcherTransformRobot,
      Pose3d fieldTargetPose,
      Translation2d fieldPlatformVelocityMps) {
    // Launcher pose in field frame
    fieldLauncherPose = fieldRobotPose.plus(launcherTransformRobot);

    // Distance to target
    shooter2d = fieldLauncherPose.toPose2d();
    hub2d = fieldTargetPose.toPose2d();
    translation = hub2d.relativeTo(shooter2d).getTranslation();
    double distance = translation.getNorm();
    double psi = translation.getAngle().getRadians();

    // This is the robot YAW; compute field-relative angle
    double yaw = fieldLauncherPose.getRotation().getZ() + Math.PI;
    double psiFieldRad = MathUtil.angleModulus(psi + yaw);

    // Shooting on the move!!!
    double vRobot = fieldPlatformVelocityMps.getNorm();
    Rotation2d robotVel2HubVectorAngle =
        vRobot > 0.0
            ? fieldPlatformVelocityMps.getAngle().minus(translation.getAngle())
            : Rotation2d.kZero;

    double radialVelocity = vRobot * Math.cos(robotVel2HubVectorAngle.getRadians());
    double projectedDistance = distance + radialVelocity * kTimeOfFlight;

    v0 = BasicRegression.computeRegression(projectedDistance);

    v0 -=
        vRobot
            * (Math.cos(robotVel2HubVectorAngle.getRadians()))
            / Math.cos(kShotAngle)
            / kFlywheelCircumfrence
            * 60.0;

    psiFieldRad -=
        vRobot * (Math.sin(robotVel2HubVectorAngle.getRadians())) * kTimeOfFlight / distance;

    if (Math.sqrt(
            Math.pow(Math.abs((fieldRobotPose.getY() - fieldTargetPose.getY())), 2)
                + Math.pow(Math.abs((fieldRobotPose.getX() - fieldTargetPose.getX())), 2))
        > 1.5) {
      return new EpicShotSolution(v0, Rotation2d.fromRadians(psiFieldRad));
    } else {
      return new EpicShotSolution(v0 + 0.05, Rotation2d.fromRadians(psiFieldRad));
    }
  }
}
