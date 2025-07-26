package frc.robot.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain.Drive;
import frc.robot.Utilities.LimeLightHelpers;

/// Subsystem that deals with the vision system in our robot

public class LimeVision implements Sendable, Subsystem {

    static Transform3d robotToCam;
    static Drive drive;

    /// all the limelights we have
    static String[] limeLights = { "limelight-one", "limelight"};

    /// current pose estimate based on the limelights
    static LimeLightHelpers.PoseEstimate poseEstimate;

    /// certain constants to filter out bad readings from the limelights
    static double minAmbiguity = 0.03;
    static double maxDistance = 2;



    /// this fetches all the important information from tags,
    /// this is done at the beginining of the robot because it takes some time.
    static {
        System.out.println("Fetching tags, please wait");
        for (int x = 1; x < (Constants.Location.APRIL_TAGS_COUNT + 1); x++) {
            getTagPose(x);
            System.out.println("Fetched tag: " + x);
        }

        System.out.println("Fetched all tags");
    }

    /// gets all the tags from the multiple limelights
    public static ArrayList<LimeLightHelpers.RawFiducial> getTags() {
        ArrayList<LimeLightHelpers.RawFiducial> tags = new ArrayList<>();

        for (String limeName : limeLights) {
            /// going through every limelight and seeing all the tag that certain one can see
            /// and adds each one of that to the tags array list
            LimeLightHelpers.RawFiducial[] tagsFromCurrentLimeLight = LimeLightHelpers.getRawFiducials(limeName);

            for (LimeLightHelpers.RawFiducial tag : tagsFromCurrentLimeLight) {
                tags.add(tag);
            }
        }

        return tags;
    }

    /// setting orientation helps the limelight localize.
    /// more of this is told about in limelight documentation
    public static void setRobotOrientation() {
        double yaw = Gyro.getYaw();
        for (String limeName : limeLights) {
            LimeLightHelpers.SetRobotOrientation(limeName, yaw, 0, 0, 0, 0, 0);
        }
    }

    /// gets all the pose estimation for the robot from multiple cameras.
    public static ArrayList<LimeLightHelpers.PoseEstimate> getPoseEstimates() {
        setRobotOrientation();

        ArrayList<LimeLightHelpers.PoseEstimate> newEstimates = new ArrayList<>();

        for (String limeName: limeLights) {

            LimeLightHelpers.PoseEstimate limeEstimate = LimeLightHelpers.getBotPoseEstimate_wpiBlue(limeName);
            /// gets the estimated pose from a certain limelight
            /// afterwards, filtering is done and if the filtering passes, it is added to all the valid estimation

            if (limeEstimate == null) continue;

            double limeHighestAmbiguity = -1;

            for (LimeLightHelpers.RawFiducial tag: limeEstimate.rawFiducials) {


                if (tag.distToRobot > maxDistance) {
                    continue;
                }
                if (tag.ambiguity > limeHighestAmbiguity) limeHighestAmbiguity = tag.ambiguity;
            }

            if (minAmbiguity > limeHighestAmbiguity) continue;
            newEstimates.add(limeEstimate);
        }

        /// all the valid estimates are returned for processing (in Drive.java)
        return newEstimates;
    }

    /// flashes the limelight?
    public static void blinkLimelights() {
        for (String limeName: limeLights) {
            LimeLightHelpers.setLEDMode_ForceBlink(limeName);
        }
    }


    /// goes through all the tags a limelight can see and sees which one is the closest
    /// not much useful, it was useful for testing earlier on.
    public static LimeLightHelpers.RawFiducial getClosestTag(boolean toCamera) {
        ArrayList<LimeLightHelpers.RawFiducial> currentTags = getTags();

        LimeLightHelpers.RawFiducial closest = null;
        double closestBy = 99999999;

        for (LimeLightHelpers.RawFiducial tag : currentTags) {
            double distance;

            if (toCamera) {
                distance = tag.distToCamera;
            } else {
                distance = tag.distToRobot;
            }
            if (distance < closestBy) {
                closestBy = distance;
                closest = tag;
            }
        }
        return closest;
    }

    public static LimeLightHelpers.RawFiducial getClosestTag() {
        return getClosestTag(false);
    }

    /// just does getTagPose but using a limelight detected tag
    public static Pose2d getTagPose(LimeLightHelpers.RawFiducial tag) {
        return getTagPose(tag.id);
    }

    /// just does getTagPose but tries to find the closest tag
    public static Pose2d getTagPose() {
        return getTagPose(getClosestTag());
    }

    /// returns the tag pose using the cached AprilTag data in constants
    public static Pose2d getTagPose(int tagNumber) {
        Pose2d pose = (Pose2d) Constants.Location.TAG_PROPERTIES.get(tagNumber);

        if (pose == null) {
            return new Pose2d();
        }

        return pose;
    }

    /// average distance to robot from a tag(s)
    public static double getAverageDistanceToRobot(LimeLightHelpers.PoseEstimate estimate) {
        double sum = 0;

        for (LimeLightHelpers.RawFiducial tag: estimate.rawFiducials) {
            sum += tag.distToRobot;
        }

        return sum / (double) estimate.rawFiducials.length;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}
