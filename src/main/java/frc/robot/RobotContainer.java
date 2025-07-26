// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.RunMotor;
import frc.robot.System.Motor;

public class RobotContainer {
  CommandXboxController controller = new CommandXboxController(0);
  Motor motorSystem = new Motor(0);
  
  // called when the robot is first initialized
  public RobotContainer() {
    configureBindings();
  }

  // configure bindings to configure the Controller's keybinds to set it up to our liking
  private void configureBindings() {
    controller.b().whileTrue(new RunMotor(motorSystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
