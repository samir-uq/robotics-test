package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.Controller;

/// This command allows the main driver to drive the robot using their joysticks

public class DriveManual extends Command {
  Controller joy;
  Drive driveSubsystem;

  /// slow rate limiter makes sure that the joystick is eased out and not instant so the robot looks smooth when accelerating and deaccelrating
  /// recommended to look more on official documentation.
  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelXYMpS2);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelXYMpS2);
  SlewRateLimiter thetaLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelThetaRpS2);
  Gyro gyro;

  public DriveManual(Drive driveSubsystem, Controller joy) {
    this.driveSubsystem = driveSubsystem;
    this.joy = joy;
    this.gyro = driveSubsystem.gyro;
    addRequirements(driveSubsystem);
  }

  public ChassisSpeeds getChassisSpeeds() {
    /// this gets the X,Y of the left jotstick, which tells us what direction we want to go
    /// and Z, which is the rotation for the robot.
    double joyX = -joy.getLeftY();
    double joyY = -joy.getLeftX();
    double joyZ = joy.getRightX();

    /// a deadband helps nullify stick drift .
    joyX = MathUtil.applyDeadband(joyX, .012);
    joyY = MathUtil.applyDeadband(joyY, .012);
    joyZ = MathUtil.applyDeadband(joyZ, .012);

    /// slew rate math to determine the right value to ease out
    /// no idea what this math does u can look more into it
    joyX = xLimiter.calculate(joyX * joyX * Math.signum(joyX));
    joyY = yLimiter.calculate(joyY * joyY * Math.signum(joyY));
    joyZ = thetaLimiter.calculate(joyZ * joyZ * Math.signum(joyZ));

    /// using all of this data, we create a new ChassisSpeed object with the said values
    /// u can see joyX*maxSpeed, this is because joyX can only go from 0-1
    return new ChassisSpeeds(joyX * Constants.Drive.maxDriveSpeedMpS, joyY * Constants.Drive.maxDriveSpeedMpS,
        joyZ * Constants.Drive.maxTurnSpeedRpS);
  }

  @Override
  public void execute() {
    /// called every frame

    ///  checks if the right bumper is being held, if so it slows the robot down heavily
    double multiplier = joy.getRightBumperButton() ? .25 : 1;

    /// based on if you are holding left bumber or not, we set the robot speed / field speed so the robot moves according to the said speed
    /// you can look more on what a robot and a field speed is on official documentation or even figure it out on the drive subsystem.
    if (joy.getLeftBumperButton()) {
      driveSubsystem.setRobotSpeeds(getChassisSpeeds().times(multiplier));
    } else {
      driveSubsystem.SetFieldSpeeds(getChassisSpeeds().times(multiplier));
    }

    /// if we hold the right stick, it zeroes the yaw.
    /// this allows for the driver to change how their x and y behave on their controllers.
    if (joy.getRightStickButton()) {
      gyro.zeroYaw();

      /// testing for vision
      driveSubsystem.testOdomWatchVision = !driveSubsystem.testOdomWatchVision;
      
    }
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(driveSubsystem);
  }
}
