package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.Supplier;

public class PathAngleOverride {

  private static Supplier<Optional<Rotation2d>> rotationOverride = Optional::empty;

  private PathAngleOverride() {}

  public static void setOverride(Supplier<Optional<Rotation2d>> supplier) {
    rotationOverride = supplier;
  }

  public static void clearOverride() {
    rotationOverride = Optional::empty;
  }

  public static Optional<Rotation2d> getOverride() {
    return rotationOverride.get();
  }
}
