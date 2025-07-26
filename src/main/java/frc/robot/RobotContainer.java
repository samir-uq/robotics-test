// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.RunMotor;
import frc.robot.Commands.SetMotor;
import frc.robot.System.Motor;

public class RobotContainer {
  // both controller and motorSystem initialize core systems of our robot code that is needed for what we r gonna do
  CommandXboxController controller = new CommandXboxController(0);
  Motor motorSystem = new Motor(0);
  
  // called when the robot is first initialized
  public RobotContainer() {
    configureBindings();

    // setDefaultCommand makes it so that the command is always running (wouldnt need it right now tho)
    // motorSystem.setDefaultCommand(command);
    // the only place where this would be handy is for whenever you are trying to have a command that puts a certain motor at a certain pos
    // or a joystick thats providing continous input (as for drive code)
  }

  // configure bindings to configure the Controller's keybinds to set it up to our liking
  private void configureBindings() {
    controller.b().whileTrue(new RunMotor(motorSystem));

    // this makes it so that whenever you press x, it will put the position of the motor to 4
    controller.x().onTrue(new SetMotor(motorSystem, 4));
  }

  // discard right now
  // used in autonomous mode to get the right command for the robot to execute
  public Command getAutonomousCommand() {
    return null;
  }
}
