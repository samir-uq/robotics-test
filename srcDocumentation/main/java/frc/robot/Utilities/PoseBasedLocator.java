package frc.robot.Utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.Drive;

/// This file is used to deal with pose related location for our robot.
/// Very important as 2025 had a lot of different places to align to and pose estimation was vevry important.

public class PoseBasedLocator {
    Drive drive;
    /// this gets our alliance during a real match
    Optional<Alliance> alliance = DriverStation.getAlliance();
    
    /// just an identifier to see if we are red or blue team.
    boolean red = false;

    public PoseBasedLocator(Drive drive) {
        this.drive = drive;
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) red = false;
        }
    }

    public int poseToId(Pose2d pose) {
        /// this converts the given pose2d to a tag if they have the same pose2d, if not it just returns 0.
        
        for (AprilTag tag: Constants.Location.APRIL_TAGS) {
            if (tag.pose.toPose2d().equals(pose)) {
                return tag.ID;
            }
        }

        return 0;
    }

    public List<Pose2d> arrayToPoseArray(int[] ids) {
        /// returns a lost of pose2d of the given april tag ids.
        /// this is just to fetch all the valid pose2d for certain type of ids.
        ArrayList<Pose2d> poses = new ArrayList<>();

        for (int id: ids) {
            Pose2d newPose = Constants.Location.APRIL_TAGS.get(id-1).pose.toPose2d();
            poses.add(newPose);
        }

        return poses;
    }

    public int getClosestCoralTagToRobot() {
        /// this gets the current estimated pose of the robot
        Pose2d robotPose = drive.getAncientPose();

        drive.states.swerveControllerTarget = robotPose;

        /// .nearest is a built in method that returns the closest Pose2d to robotPose out of all the Pose2d
        /// COnstants.Location.REEF_IDS is a List<Pose2d>
        /// the returned object is also a pose, which is ran through poseToId to get the valid ID.
        Pose2d closest = robotPose.nearest(arrayToPoseArray(Constants.Location.REEF_IDS));
        int possibleNewId = poseToId(closest);

        /// this is used to make sure other sides tags are not fetched.
        if (red && possibleNewId > 11) return 0;
        return possibleNewId;
    }


    /// since there are only 2 possible algae station pose, you dont really need estimation.
    public int getClosestAlgaeStation() {
        return red ? 3 : 16;
    }


    /// same thing as getClosestCoralTagToRobot but instead it has a different set of Poses for different points we need to go to.
    public int getClosestReefStationToRobot() {
        Pose2d robotPose = drive.getAncientPose();

        drive.states.swerveControllerTarget = robotPose;

        Pose2d closest = robotPose.nearest(arrayToPoseArray(Constants.Location.CORAL_STATION_IDS));
        int possibleNewId = poseToId(closest);

        if (red && possibleNewId > 11) return 0;
        return possibleNewId;
    }

}
