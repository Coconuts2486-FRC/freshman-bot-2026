// Copyright (c) 2026
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.extender;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;

public class ExtenderIOPCM implements ExtenderIO {

  private static final int kPcmCanId = 24;
  private static final int kBlockerSolenoidChannel = 7;

  // Own one PCM instance so its output and fault telemetry can be logged alongside the mechanism.
  private final PneumaticsControlModule pcm = new PneumaticsControlModule(kPcmCanId);

  // Creating the compressor/solenoid enables the PCM's digital closed-loop compressor control.
  public final Compressor m_compressor = pcm.makeCompressor();

  // The solenoid for the blocker is in port #7 on the PCM.
  public final Solenoid m_solenoid = pcm.makeSolenoid(kBlockerSolenoidChannel);

  // IMPORTANT: Include here all devices listed above that are part of this mechanism!
  public final int[] powerPorts = {};

  @Override
  public int[] powerPorts() {
    return powerPorts;
  }

  // Define any status signals here that you need

  // Constructor
  public ExtenderIOPCM() {}

  @Override
  public void updateInputs(ExtenderIOInputs inputs) {
    inputs.compressorEnabled = pcm.getCompressor();
    inputs.pressureSwitchLow = pcm.getPressureSwitch();
    inputs.compressorCurrentAmps = pcm.getCompressorCurrent();
    inputs.compressorConfig = pcm.getCompressorConfigType().toString();

    inputs.compressorCurrentTooHighFault = pcm.getCompressorCurrentTooHighFault();
    inputs.compressorCurrentTooHighStickyFault = pcm.getCompressorCurrentTooHighStickyFault();
    inputs.compressorShortedFault = pcm.getCompressorShortedFault();
    inputs.compressorShortedStickyFault = pcm.getCompressorShortedStickyFault();
    inputs.compressorNotConnectedFault = pcm.getCompressorNotConnectedFault();
    inputs.compressorNotConnectedStickyFault = pcm.getCompressorNotConnectedStickyFault();
    inputs.compressorConnected = !inputs.compressorNotConnectedFault;

    inputs.blockerSolenoidEnabled = m_solenoid.get();
    inputs.solenoidOutputMask = pcm.getSolenoids();
    inputs.solenoidDisabledMask = pcm.getSolenoidDisabledList();
    inputs.blockerSolenoidDisabled =
        (inputs.solenoidDisabledMask & (1 << kBlockerSolenoidChannel)) != 0;
    inputs.solenoidVoltageFault = pcm.getSolenoidVoltageFault();
    inputs.solenoidVoltageStickyFault = pcm.getSolenoidVoltageStickyFault();
  }

  @Override
  public void extendBlocker() {
    // Set to true to activate the solenoid, extending the blocker
    m_solenoid.set(true);
  }

  @Override
  public void retractBlocker() {
    // Set to false to deactivate the solenoid, retracting the blocker
    m_solenoid.set(false);
  }
}
