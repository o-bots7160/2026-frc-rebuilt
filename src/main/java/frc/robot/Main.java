// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Entry point that launches the WPILib robot framework.
 */
public final class Main {
  /**
   * Starts the robot application using WPILib's lifecycle.
   *
   * @param args command-line arguments (unused)
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }

  /**
   * Prevents instantiation of this utility class.
   */
  private Main() {}
}
