package frc.robot.Subsystems.Drivetrain;

import java.util.Calendar;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface IDriveIO {
    /// more about IO and AutoLog on ICageIO.java


    /// Also, drivetrain is always the most important part of the robot
    /// therefore, the logs for them will always be way bigger than the rest of them
    /// most of these are useless but also important depending on specific times.
    @AutoLog
    public class DriveStates {
        double gyroAngleDeg;
        double[] odomPoses = { 0, 0, 0 };
        double[] lastCachedOdomPosePoint = { 0, 0, 0 };

        double[] chassisSpeeds = new double[3];

        double[] targetSpeeds = { 0, 0, 0 };
        double[] targetAutoSpeeds = { 0, 0, 0 };

        boolean recievedNewControls;

        double cacheTime = 0;
        double lastUpdatedTime = (double) (Calendar.getInstance().getTimeInMillis() / 1000);
        Pose2d position;

        public Pose2d swerveControllerTarget;
        public Pose2d targetCache;
        public Pose2d driveToPoitnEstimate;        

        public double differenceX = 0;
        public double differenceY = 0;
        public double differenceHeading = 0;
    }

    public void updateDriveStates(DriveStatesAutoLogged states);
}
