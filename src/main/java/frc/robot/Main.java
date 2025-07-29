// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/// As per usual, theres always a main file that runs the main execution of the robot.
/// All this main class does is create a new robot and start it with the "RobotBase" library

public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    /// Syntax Robot::new means you are sending the Robot.new() method
    /// RobotBase.startRObot() starts the robot
    /// The main reason you don't have a new Robot() here and instead a supplier and RobotBase is because inside the RIO
    /// Rio: the computer of the robot
    /// Inside the RIO, you can restart your robot and other things. Therefore this provides a more fluid way to code the robot.
    RobotBase.startRobot(Robot::new);

    /// If unknown, Robot::new refers to the Robot.java file
  }
}
