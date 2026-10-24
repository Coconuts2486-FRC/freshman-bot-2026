import java.nio.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import org.wpilib.datalog.*;

/** Summarizes driver-control, alliance, and odometry behavior from WPILOG files. */
public class AnalyzeElims {
  record Event(double sec, String text) {}

  static Object decode(String type, DataLogRecord record) {
    if (type.equals("boolean")) return record.getBoolean();
    if (type.equals("int64")) return record.getInteger();
    if (type.equals("float[]")) return record.getFloatArray();
    if (type.equals("string")) return new String(record.getRaw(), StandardCharsets.UTF_8);
    ByteBuffer buffer = record.getRawBuffer().order(ByteOrder.LITTLE_ENDIAN);
    if (type.equals("struct:Rotation2d")) return buffer.getDouble();
    if (type.equals("struct:ChassisSpeeds")) {
      return new double[] {buffer.getDouble(), buffer.getDouble(), buffer.getDouble()};
    }
    if (type.equals("struct:Pose2d")) {
      return new double[] {buffer.getDouble(), buffer.getDouble(), buffer.getDouble()};
    }
    return null;
  }

  static double wrap(double angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }

  static String vector(double[] values) {
    return String.format(Locale.ROOT, "[%.2f, %.2f, %.2f]", values[0], values[1], values[2]);
  }

  public static void main(String[] args) throws Exception {
    for (String filename : args) {
      analyze(filename);
    }
  }

  static void analyze(String filename) throws Exception {
    var names =
        Map.of(
            "joystick", "/DriverStation/Joystick0/AxisValues",
            "enabled", "/DriverStation/Enabled",
            "auto", "/DriverStation/Autonomous",
            "alliance", "/DriverStation/AllianceStation",
            "yaw", "/RealOutputs/Odometry/Yaw",
            "target", "/RealOutputs/SwerveChassisSpeeds/Setpoints",
            "measured", "/RealOutputs/SwerveChassisSpeeds/Measured",
            "pose", "/RealOutputs/Odometry/Robot",
            "resetEpoch", "/RealOutputs/Drive/PoseResetEpoch",
            "driveStyle", "/NetworkInputs/SmartDashboard/Drive Style");
    var reader = new DataLogReader(filename);
    Map<Integer, DataLogRecord.StartRecordData> entries = new HashMap<>();
    float[] axes = new float[6];
    double[] target = new double[3], measured = new double[3], pose = new double[3];
    boolean enabled = false, auto = false;
    long alliance = -1, epoch = -1;
    double yaw = 0, base = -1;
    String driveStyle = "";
    List<Event> events = new ArrayList<>();
    int samples = 0, inputOver90 = 0, measuredOver90 = 0;
    double inputErrorSum = 0, measuredErrorSum = 0, maxInputError = 0, maxMeasuredError = 0;
    String truncation = null;

    try {
      for (var record : reader) {
        if (record.isStart()) {
          var entry = record.getStartData();
          entries.put(entry.entry, entry);
          continue;
        }
        var entry = entries.get(record.getEntry());
        if (entry == null) continue;
        Object value = decode(entry.type, record);
        if (value == null) continue;
        if (base < 0) base = record.getTimestamp() / 1e6;
        double seconds = record.getTimestamp() / 1e6 - base;

        if (entry.name.equals(names.get("joystick"))) axes = (float[]) value;
        else if (entry.name.equals(names.get("yaw"))) yaw = (double) value;
        else if (entry.name.equals(names.get("measured"))) measured = (double[]) value;
        else if (entry.name.equals(names.get("pose"))) pose = (double[]) value;
        else if (entry.name.equals(names.get("target"))) {
          target = (double[]) value;
          if (enabled && !auto && axes.length >= 2) {
            double fieldX = -axes[1], fieldY = -axes[0];
            double inputMagnitude = Math.hypot(fieldX, fieldY);
            double targetMagnitude = Math.hypot(target[0], target[1]);
            if (inputMagnitude > 0.35 && targetMagnitude > 1.0) {
              // DriverStation AllianceStation values: Red = 1-3, Blue = 4-6.
              double controlHeading = yaw + (alliance >= 1 && alliance <= 3 ? Math.PI : 0.0);
              double expectedX = Math.cos(controlHeading) * fieldX + Math.sin(controlHeading) * fieldY;
              double expectedY = -Math.sin(controlHeading) * fieldX + Math.cos(controlHeading) * fieldY;
              double inputError =
                  Math.abs(
                      wrap(
                          Math.atan2(target[1], target[0])
                              - Math.atan2(expectedY, expectedX)));
              inputErrorSum += inputError;
              maxInputError = Math.max(maxInputError, inputError);
              if (inputError > Math.PI / 2) inputOver90++;
              samples++;

              double measuredMagnitude = Math.hypot(measured[0], measured[1]);
              if (measuredMagnitude > 0.6) {
                double measuredError =
                    Math.abs(
                        wrap(
                            Math.atan2(measured[1], measured[0])
                                - Math.atan2(target[1], target[0])));
                measuredErrorSum += measuredError;
                maxMeasuredError = Math.max(maxMeasuredError, measuredError);
                if (measuredError > Math.PI / 2) measuredOver90++;
              }
            }
          }
        } else if (entry.name.equals(names.get("enabled"))) {
          boolean old = enabled;
          enabled = (boolean) value;
          if (old != enabled) {
            events.add(new Event(seconds, "enabled=" + enabled + " auto=" + auto));
          }
        } else if (entry.name.equals(names.get("auto"))) {
          boolean old = auto;
          auto = (boolean) value;
          if (old != auto) events.add(new Event(seconds, "autonomous=" + auto));
        } else if (entry.name.equals(names.get("alliance"))) {
          alliance = (long) value;
          events.add(new Event(seconds, "allianceStation=" + alliance));
        } else if (entry.name.equals(names.get("resetEpoch"))) {
          epoch = (long) value;
          events.add(
              new Event(
                  seconds,
                  "poseResetEpoch="
                      + epoch
                      + " yaw="
                      + Math.toDegrees(yaw)
                      + " pose="
                      + vector(pose)));
        } else if (entry.name.equals(names.get("driveStyle"))) {
          driveStyle = (String) value;
        }
      }
    } catch (RuntimeException exception) {
      truncation = exception.getMessage();
    }

    System.out.println("\n" + Paths.get(filename).getFileName());
    System.out.printf(
        Locale.ROOT,
        "  driveStyle=%s; samples=%d; input direction mean/max=%.1f/%.1f deg; >90deg=%d%n",
        driveStyle,
        samples,
        Math.toDegrees(samples == 0 ? 0 : inputErrorSum / samples),
        Math.toDegrees(maxInputError),
        inputOver90);
    System.out.printf(
        Locale.ROOT,
        "  measured-vs-command direction mean/max=%.1f/%.1f deg; >90deg=%d%n",
        Math.toDegrees(samples == 0 ? 0 : measuredErrorSum / samples),
        Math.toDegrees(maxMeasuredError),
        measuredOver90);
    for (var event : events) System.out.printf(Locale.ROOT, "  %7.3fs %s%n", event.sec, event.text);
    if (truncation != null) System.out.println("  NOTE: malformed trailing record; prior samples analyzed.");
  }
}
