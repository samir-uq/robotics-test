package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain.Drive;

import com.pathplanner.lib.auto.AutoBuilder;

/// Command used by the FRC Pathplanner autobuilder to get the right command for auto.
/// more about FRC pathplanner was on README.md

public class FollowPath {
  static Timer pathTimer = new Timer();
  public static SendableChooser<Command> autoChooser;

  public static void updateAutoSelector(Drive drive) {
    autoChooser = AutoBuilder.buildAutoChooser();

    /// this puts the option to choose auto in our smart dashboard
    /// using this, we set our auto before every game to make sure we run the right auto.
    SmartDashboard.putData(autoChooser);
  }

}
