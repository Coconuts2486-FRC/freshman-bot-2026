package frc.robot.subsystems.extender;

import frc.robot.util.RBSIIO;
import org.littletonrobotics.junction.AutoLog;

public interface ExtenderIO extends RBSIIO {
  @AutoLog
  public static class ExtenderIOInputs {
    /** True when the PCM reports that the compressor motor is connected. */
    public boolean compressorConnected = false;

    /** True while the PCM is powering the compressor output. */
    public boolean compressorEnabled = false;

    /** True when the pressure switch reports that the system needs more pressure. */
    public boolean pressureSwitchLow = false;

    public double compressorCurrentAmps = 0.0;
    public String compressorConfig = "Disabled";

    public boolean compressorCurrentTooHighFault = false;
    public boolean compressorCurrentTooHighStickyFault = false;
    public boolean compressorShortedFault = false;
    public boolean compressorShortedStickyFault = false;
    public boolean compressorNotConnectedFault = false;
    public boolean compressorNotConnectedStickyFault = false;

    /** Requested state of the blocker solenoid (PCM channel 7). */
    public boolean blockerSolenoidEnabled = false;

    /** PCM output state bitmask; bit 0 is solenoid channel 0. */
    public int solenoidOutputMask = 0;

    /** PCM disabled-solenoid bitmask; bit 0 is solenoid channel 0. */
    public int solenoidDisabledMask = 0;

    public boolean blockerSolenoidDisabled = false;
    public boolean solenoidVoltageFault = false;
    public boolean solenoidVoltageStickyFault = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ExtenderIOInputs inputs) {}

  public default void extendBlocker() {}

  public default void retractBlocker() {}
}
