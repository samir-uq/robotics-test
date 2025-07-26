// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveManual;
import frc.robot.Commands.FollowPath;
import frc.robot.Commands.SimpleCoralAlign;
import frc.robot.Subsystems.LimeVision;
import frc.robot.Subsystems.Algae.Algae;
import frc.robot.Subsystems.Algae.AlgaeDescore;
import frc.robot.Subsystems.Cage.Cage;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.Controller;
import frc.robot.Utilities.PoseBasedLocator;
import frc.robot.Utilities.Controller.vibrationScalar;
import frc.robot.Utilities.Controller.vibrationType;
import frc.robot.Commands.AlgaeIntake;
import frc.robot.Commands.AlgaeScore;
import frc.robot.Commands.AutoDescoreAlgae;
import frc.robot.Commands.CoralIntake;
import frc.robot.Commands.CoralIntakeOutake;
import frc.robot.Commands.CoralL1intake;
import frc.robot.Commands.CoralL1score;
import frc.robot.Commands.CoralScore;
import frc.robot.Commands.Corall1;
import frc.robot.Commands.DescoreAlgae;
import frc.robot.Commands.CageManual;
import frc.robot.Commands.ElevatorPositon;

/// Robot Container is where we handle the main logic, this class is initiated by Robot.java

public class RobotContainer {
  /// For Vibrating the controller at the last 30 sec
  private boolean endMatchControllerVibrationSent = false;

  /// Controller is our own wrapper class that helps us deal with our XBOX Controllers. Its under Utils
  /// Getting the Controller of the main Driver
  private Controller joyDrive = new Controller(0);
  /// Getting the controller of the Opperator
  private Controller joyOpperator = new Controller(1);

  /// ALl the subsystems related to the robot being initiated.
  /// You can see that some of them have a parameter which is the id of the motor required for them to run properly.
  private Drive drive = new Drive();
  private static CoralElevator elevator = new CoralElevator(Constants.Coral.elevatorCANID);
  private static CoralEndEffector effector = new CoralEndEffector(Constants.Coral.endEffectorCANID);
  private static Algae algae = new Algae(Constants.Algae.anglerCANID, Constants.Algae.intakeCANID);
  private static AlgaeDescore descore = new AlgaeDescore(Constants.Algae.descoreCANID);
  private static Cage cage = new Cage(Constants.Cage.cageCANID);

  /// Pose Based Locator is an Utility that helps us find stuff in the field using our Pose Estimation.
  private PoseBasedLocator poseLocator = new PoseBasedLocator(drive);

  /// These are all Commands that are used by the drivers to operate the robot
  private DriveManual teleopDriveCmd = new DriveManual(drive, joyDrive);
  private CageManual cageman = new CageManual(cage, joyOpperator);
  private Corall1 L1 = new Corall1(algae, joyDrive);
  

  // TODO add 888's dashboard
  /// Camera to see the cage
  @SuppressWarnings("unused")
  private UsbCamera camera = CameraServer.startAutomaticCapture();


  public RobotContainer() {
    /// Bindings: Buttons of each Controller
    configureBindings();
    drive.log();

    drive.manualDrive = teleopDriveCmd;
    drive.poseBasedLocator = poseLocator;

    /// setting default command means its always the default command being executed unless there is another one thats prioritized.
    drive.setDefaultCommand(teleopDriveCmd);
    algae.setDefaultCommand(L1);
    cage.setDefaultCommand(cageman); 

    /// named commands for auto
    configureNamedCommands();

    /// This sets the new auto to run for our robot
    FollowPath.updateAutoSelector(drive);
    endMatchControllerVibrationSent = false;
  }

  private void configureNamedCommands() {

    // auto

    /// All of these commands link a Name to a certain Command object, this is how we utilize customized functions in our auto section.
    NamedCommands.registerCommand("LeftAlign", new SimpleCoralAlign(drive, true));
    NamedCommands.registerCommand("RightAlign", new SimpleCoralAlign(drive, false));

    NamedCommands.registerCommand("IntakePosition", new ElevatorPositon(elevator, Constants.Coral.intakePos));

    NamedCommands.registerCommand("CoralScoreL1", new CoralScore(elevator, effector, Constants.Coral.l1Pos));
    NamedCommands.registerCommand("CoralScoreL2", new CoralScore(elevator, effector, Constants.Coral.l2Pos));
    NamedCommands.registerCommand("CoralScoreL3", new CoralScore(elevator, effector, Constants.Coral.l3Pos));

    NamedCommands.registerCommand("CoralL1", new ElevatorPositon(elevator, Constants.Coral.l1Pos));
    NamedCommands.registerCommand("CoralL2", new ElevatorPositon(elevator, Constants.Coral.l2Pos));
    NamedCommands.registerCommand("CoralL3", new ElevatorPositon(elevator, Constants.Coral.l3Pos));

    NamedCommands.registerCommand("YieldIntake", new CoralIntake(elevator, effector, joyOpperator));

    NamedCommands.registerCommand("DescoreStart", new AutoDescoreAlgae(descore, -0.7));
    NamedCommands.registerCommand("DescoreStop", new AutoDescoreAlgae(descore, 0));

  }

  public Command getAutonomousCommand() {
    /// basically just returns the right Command to run at the very moment for our autonomous mode.
    /// Look more into the FollowPath.java or AutoBuilder/AutoChooser
    return FollowPath.autoChooser.getSelected();
  }

  public void periodic() {
    /// this is a redundant feature that was added at the very last second to make sure there was a buzz on both drivers controller at the last 30 sec
    /// this would not be the optimal way to do it.
    if (DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() <= Constants.Robot.endGameControllerVibrationTimer && DriverStation.isTeleopEnabled() && !endMatchControllerVibrationSent) {
      endMatchControllerVibrationSent = true;
      joyDrive.vibrate(vibrationScalar.HIGH, 10, vibrationType.BOTH);
      joyOpperator.vibrate(vibrationScalar.HIGH, 10, vibrationType.BOTH);
      LimeVision.blinkLimelights();
    }

  }

  private void configureBindings() {
    /// Bindings for each controller
    /// Look more into Controller.java and XboxController library in WPILIB to see how these work
    /// as well as Trigger, they are really useful to bind commands to a certain button.
    /// It shouldnt be that difficult to grasp onto what each of these means
    /// debouce is just like a wait period before it is called again
    /// look more into specific command to see how they function.

    // left and right to reef
    new Trigger(() -> joyDrive.getXButton()).debounce(.1).whileTrue(new SimpleCoralAlign(drive, true));
    new Trigger(() -> joyDrive.getBButton()).debounce(.1).whileTrue(new SimpleCoralAlign(drive, false));

    new Trigger(() -> joyDrive.getRawButton(7)).debounce(.1).whileTrue(new SimpleCoralAlign(drive, true));
    new Trigger(() -> joyDrive.getRawButton(8)).debounce(.1).whileTrue(new SimpleCoralAlign(drive, false));

    new Trigger(() -> joyDrive.getYButton()).debounce(.1).whileTrue(new SimpleCoralAlign(drive, false, 2));//find button


    // center coral station
    new Trigger(() -> joyDrive.getAButton()).debounce(.1).onTrue(new SimpleCoralAlign(drive, false, 1));

    new Trigger(() -> joyDrive.getLeftTriggerAxis() > 0.2).debounce(.1).whileTrue(new AlgaeIntake(algae));
    new Trigger(() -> joyDrive.getRightTriggerAxis() > 0.2).debounce(.1).whileTrue(new AlgaeScore(algae));

    new Trigger(() -> joyOpperator.getLeftBumperButton()).debounce(.1).whileTrue(new CoralL1intake(algae));
    new Trigger(() -> joyOpperator.getRightBumperButton()).debounce(.1).whileTrue(new CoralL1score(algae));

    

    new Trigger(() -> joyOpperator.getPOV() == 270).debounce(.1).whileTrue(new DescoreAlgae(descore, -0.5));
    new Trigger(() -> joyOpperator.getPOV() == 90).debounce(.1).whileTrue(new DescoreAlgae(descore, 0.5));



    new Trigger(() -> joyOpperator.getXButton()).debounce(.1).onTrue(new CoralIntakeOutake(effector, joyOpperator));
    new Trigger(() -> joyOpperator.getAButton()).debounce(.1)
        .onTrue(new ElevatorPositon(elevator, Constants.Coral.l1Pos));
    new Trigger(() -> joyOpperator.getBButton()).debounce(.1)
        .onTrue(new ElevatorPositon(elevator, Constants.Coral.l2Pos));
    new Trigger(() -> joyOpperator.getYButton()).debounce(.1)
        .onTrue(new ElevatorPositon(elevator, Constants.Coral.l3Pos));
  }

}
