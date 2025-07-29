package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.LimeVision;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.Controller;
import frc.robot.Utilities.Controller.vibrationScalar;
import frc.robot.Utilities.Controller.vibrationType;

/// Command to align our robot to the right Pose2d

public class SimpleCoralAlign extends Command {
    
    /// target where we want to be, our drive train
    private Pose2d target;
    private Drive drive;

    /// boolean left signifies if we are aligning to the left or the right side of the reef, if we are aligning to reef
    private boolean left;
    /// alignType states what type of align it is, there are three types of align, reef, station and reef (l1)
    private int alignType;

    /// this timer is used to time the duration of this command for auto, because as I said, in auto you cant risk yyielding for the whole 15 second.
    private Timer timer = new Timer();


    /// controllers of both drivers
    /// this is used to make it vibrate based on the align.
    private Controller joyDrive = new Controller(0);
    private Controller joyOperator = new Controller(1);
    private vibrationType vibratingSide;


    /// These are PIDs that are used to calculate the right moving and rotating speed to get to our target.
    private ProfiledPIDController driveController = new ProfiledPIDController(
        Constants.Drive.DriveToPoint.kDriveToPointP,
        Constants.Drive.DriveToPoint.kDriveToPointI,
        Constants.Drive.DriveToPoint.kDriveToPointD,
        new TrapezoidProfile.Constraints(
            Constants.Drive.DriveToPoint.kMaxLinearSpeed, Constants.Drive.DriveToPoint.kMaxLinearAcceleration));
  
      private ProfiledPIDController headingController = new ProfiledPIDController(
        Constants.Drive.DriveToPoint.kDriveToPointHeadingP,
        Constants.Drive.DriveToPoint.kDriveToPointHeadingI,
        Constants.Drive.DriveToPoint.kDriveToPointHeadingD,
        new TrapezoidProfile.Constraints(
            Constants.Drive.DriveToPoint.kMaxAngularSpeed, Constants.Drive.DriveToPoint.kMaxAngularAcceleration));
  

    ///feed forward is apart of the pid thing, it states how fast you can go relative to your distance to the target
    /// knowledge on PID is a must, you should look into ProfilePIDController very extensively.
    public double ffMinRadius = 100.0;
    public double ffMaxRadius = 100.0;
            

    public SimpleCoralAlign(Drive drive, boolean left, int alignType) {
        this.drive = drive;
        this.left = left;
        this.alignType = alignType;


        /// depending on what side we align (if we even aling to reef), we set the vibration type to a different motor
        /// this is so the drivers dont have to communicate which side they are alinigng to and can easily know based on the motors cibrating
        /// 
        if (left) {
            vibratingSide = vibrationType.LEFT;
        } else {
            vibratingSide = vibrationType.RIGHT;
        }

        /// this is because alignType == 1 means Station and alignType == 2 means L1, which dont have different sides.
        if (alignType > 0) {
            vibratingSide = vibrationType.BOTH;
        }

        /// pid stuff again (just says it goes from 0 to 360, after 360 its back to 0)
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        /// setTolerance is the minimum error the pid can have before stopping.
        /// more tolerance, more innacurate
        driveController.setTolerance(Constants.Drive.DriveToPoint.metersTolerance, Constants.Drive.DriveToPoint.metersAccelTolerance);
        headingController.setTolerance(Constants.Drive.DriveToPoint.radiansTolerance, Constants.Drive.DriveToPoint.rotAccelTolerance);

        addRequirements(drive);
    }

    public SimpleCoralAlign(Drive drive, boolean left) {
        this(drive, left, 0);
        /// this just is an overloaded constructor, basic java
    }

    @Override
    public void initialize() {
        /// based on the align type, we set the target
        if (alignType == 1) {
            /// find the closest station pose using our PoseBasedLocator.java
            /// sets that as the target
            int closestStationTag = drive.poseBasedLocator.getClosestReefStationToRobot();
            Pose2d closestStationPose = LimeVision.getTagPose(closestStationTag);

            target = closestStationPose.plus(Constants.Location.CORAL_STATION_CENTER_OFFSET);
        } else {
            /// find the closest reef using PoseBasedLocator.java
            int closestCoralTag = drive.poseBasedLocator.getClosestCoralTagToRobot();
            Pose2d closestCoralPose = LimeVision.getTagPose(closestCoralTag);

            if (alignType == 2) {
                /// if we are doing L1, we set the target as the closest tag plus the center reef offset
                target = closestCoralPose.plus(Constants.Location.CENTER_REEF_OFFSET);
            } else {
                /// if we are doing l2/l3 with the elevator, we set the tag plus offset based on weather we are left or right
                if (left) {
                    target = closestCoralPose.plus(Constants.Location.FRONT_REEF_LEFT_OFFSET);
                } else {
                    target = closestCoralPose.plus(Constants.Location.FRONT_REEF_RIGHT_OFFSET);
                }
            }
        }

        /// gets the current estimated pose
        Pose2d current = drive.getPose();

        /// gets the current fieldReletaivespeeds
        ChassisSpeeds fieldRSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            drive.getChassisSpeeds(), current.getRotation());

            /// resets the driving PID
            driveController.reset(
                current.getTranslation().getDistance(target.getTranslation()),
                Math.min(
                    0.0,
                    -new Translation2d(fieldRSpeeds.vxMetersPerSecond, fieldRSpeeds.vyMetersPerSecond)
                        .rotateBy(
                            target
                                .getTranslation()
                                .minus(current.getTranslation())
                                .getAngle()
                                .unaryMinus())
                        .getX()));
            
            /// resets the rotating PID to be at 0 according to our current angle.
            headingController.reset(
                current.getRotation().getRadians(), fieldRSpeeds.omegaRadiansPerSecond
            );

        this.drive.alignCommandCache = this;
        
        /// starts vibration so the drivers know we are aligning.
        joyDrive.setVibration(joyDrive.enumToRumbleValue(vibrationScalar.HIGH), joyDrive.enumToRumbleType(vibratingSide));
        joyOperator.setVibration(joyOperator.enumToRumbleValue(vibrationScalar.HIGH), joyOperator.enumToRumbleType(vibratingSide));
        
        /// resets the timer
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (isFinished()) return;

        /// this is all math and other stuff to get the right speeds
        /// this was fetched from other teams
        /// you can go through this and explore Pose2d and other methods to figure out how this works
        /// it is not necesarry to know but it stil is very important to get the general idea.
        
        Pose2d current = drive.getPose();
        drive.states.targetCache = target;

        double currentDistance = current.getTranslation().getDistance(target.getTranslation());
        double ffScaler = MathUtil.clamp(
        (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
        + driveController.calculate(currentDistance, 0.0);
        
        if (currentDistance < driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        double headingError = current.getRotation().minus(target.getRotation()).getRadians();
        double headingVelocity = headingController.getSetpoint().velocity * ffScaler
        + headingController.calculate(
            current.getRotation().getRadians(), target.getRotation().getRadians());

        if (Math.abs(headingError) < headingController.getPositionTolerance()) {
            headingVelocity = 0.0;
        }

        Translation2d driveVelocity = new Pose2d(
            0.0,
            0.0,
            current.getTranslation().minus(target.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), -headingVelocity, current.getRotation());

        drive.setRobotSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        /// if in auto, checks if the timer is more than 2 and returns if it is so we can end it even if not.
        if (DriverStation.isAutonomous() && timer.hasElapsed(2)) return true;
    
        /// returns it as well when both the drive and rotating pid are at their goal (under the tolerance)
        return driveController.atGoal() && headingController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        /// stops controller vibration and sets the speed of the robot to 0 so it is stationary
        drive.setRobotSpeeds(new ChassisSpeeds(0,0,0));

        joyDrive.setVibration(0, joyDrive.enumToRumbleType(vibratingSide));
        joyOperator.setVibration(0, joyOperator.enumToRumbleType(vibratingSide));
    }

}
