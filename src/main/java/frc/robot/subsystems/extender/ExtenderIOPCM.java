// Copyright (c) 2026
// http://github.com/AZ-First
// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the AdvantageKit-License.md file
// at the root directory of this project.

package frc.robot.subsystems.extender;

public class ExtenderIOPCM implements ExtenderIO {

  // Define the hardware from the RobotDevices section of RobotContainer

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
    // Include here any inputs you have

  }

  @Override
  public void extendBlocker() {
    // Do something!
  }

  @Override
  public void retractBlocker() {
    // Do something!
  }
}
