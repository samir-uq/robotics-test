// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utilities.AutoCommandMap;


/// This class extends LoggedRobot, which contains a lot of the base features

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Documentation on logging
    // (https://frc3748.github.io/Code-Team-Guide/SwerveDrive/logging/)
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    /// isReal() refers to if the robot is real, this is beacuse WPILIB allows us to replay certain games as well as Simulations
    /// More oabout the Logger could be figured out through research and official documentation
    
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }

    // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic
    // Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    AutoCommandMap.mapCmds();

    // creates a Robot container, which is the main object where we handle our robot logic.
    m_robotContainer = new RobotContainer();
  }

  /// robotPeriodic is called every "frame" for the robot. 

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (m_robotContainer != null) {
      m_robotContainer.periodic();

    }    
  }

  /// all of these methods are self explanatory and should be looked into through official docs if you are unable to understand

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    try {
      CommandScheduler.getInstance().cancelAll();
    } finally {

    }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
