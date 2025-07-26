package frc.robot.Subsystems.Drivetrain;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Commands.DriveManual;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.LimeVision;
import frc.robot.Utilities.LimeLightHelpers;
import frc.robot.Utilities.PoseBasedLocator;

/// I would recommend to look at other subsystems beside Drivetrain before exploring drivetrain
/// it is the most difficult one to understand and requires decent comprehension of WPILIB and FRC coding.

/// Documentation from previous years
/// (https://frc3748.github.io/Code-Team-Guide/SwerveDrive/swerveMath/)


public class Drive implements IDriveIO, Subsystem, Sendable {
    public List<Module> modules = new ArrayList<>();
    public Gyro gyro;
    SwerveDrivePoseEstimator poseEstimator;
    boolean flipPoseEstimatorAngle = false;

    double[] blankOdomPose = { 0, 0, 0 };

    public DriveManual manualDrive;
    public PoseBasedLocator poseBasedLocator;

    public Command alignCommandCache;

    public DriveStatesAutoLogged states = new DriveStatesAutoLogged();

    public boolean testOdomWatchVision = true;

    public Drive() {
        modules.add(new Module(
                Constants.Drive.frDriveCANID,
                Constants.Drive.frTurnCANID,
                Constants.Drive.frcanCoderID,
                Constants.Module.frRadius,
                Constants.Module.frAngleOffset,
                true,
                "FR Module"));
        modules.add(new Module(
                Constants.Drive.flDriveCANID,
                Constants.Drive.flTurnCANID,
                Constants.Drive.flcanCoderID,
                Constants.Module.flRadius,
                Constants.Module.flAngleOffset,
                false,
                "FL Module"));
        modules.add(new Module(
                Constants.Drive.brDriveCANID,
                Constants.Drive.brTurnCANID,
                Constants.Drive.brcanCoderID,
                Constants.Module.brRadius,
                Constants.Module.brAngleOffset,
                true,
                "BR Module"));
        modules.add(new Module(
                Constants.Drive.blDriveCANID,
                Constants.Drive.blTurnCANID,
                Constants.Drive.blcanCoderID,
                Constants.Module.blRadius,
                Constants.Module.blAngleOffset,
                false,
                "BL Module"));
        gyro = new Gyro(50);
        resetGyro();
        SwerveDriveKinematics kinematics = Constants.Drive.kinematics;

        /// creates a pose estimator that takes in a lot of data and tries to find the right pose of the robot.
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getGyroAngle(),
                getModPositions(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                VecBuilder.fill(.025, .025, Units.degreesToRadians(.05)),
                VecBuilder.fill(.2, .2, Units.degreesToRadians(0.2)));

        /// configuring the data for FRC Pathplanner
        AutoBuilder.configure(
                this::getPose,
                this::resetPoseEstimator,
                this::getChassisSpeeds,
                this::setPathPlannerRobotSpeeds,
                new PPHolonomicDriveController(
                        Constants.Auto.xyController, Constants.Auto.thetaController),
                Constants.Auto.ppConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    @Override
    public void periodic() {

        gyro.testPigeon();
        for (Module mod : modules) {
            mod.periodic();
        }
        /// updates state and pose
        updateDriveStates(states);
        updatePoseEstimator();
        Logger.processInputs("DriveTrain", states);
    }

    public void updateDriveStates(DriveStatesAutoLogged states) {
        states.gyroAngleDeg = getGyroAngle().getDegrees();

        ChassisSpeeds speeds = getChassisSpeeds();
        states.chassisSpeeds[0] = speeds.vxMetersPerSecond;
        states.chassisSpeeds[1] = speeds.vyMetersPerSecond;
        states.chassisSpeeds[2] = speeds.omegaRadiansPerSecond;

        double[] _odomPose = {
                poseEstimator.getEstimatedPosition().getX(),
                poseEstimator.getEstimatedPosition().getY(),
                poseEstimator.getEstimatedPosition().getRotation().getRadians()
        };
        states.odomPoses = _odomPose;
    }
    /// setting speed of the robot relative to the field
    public void SetFieldSpeeds(ChassisSpeeds fieldSpeeds) {
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, getGyroAngle());
        setRobotSpeeds(robotSpeeds);
    }

    /// setting speed of the robot
    public void setRobotSpeeds(ChassisSpeeds robotSpeeds) {
        states.targetSpeeds[0] = robotSpeeds.vxMetersPerSecond;
        states.targetSpeeds[1] = robotSpeeds.vyMetersPerSecond;
        states.targetSpeeds[2] = robotSpeeds.omegaRadiansPerSecond;

        for (Module mod : modules) {
            setModuleVelocities(mod, robotSpeeds);
        }
    }
    /// setting speed of the robot for FRC Pathplanner / auto
    public void setPathPlannerRobotSpeeds(ChassisSpeeds robotSpeeds) {
        states.targetAutoSpeeds[0] = robotSpeeds.vxMetersPerSecond;
        states.targetAutoSpeeds[1] = robotSpeeds.vyMetersPerSecond;
        states.targetAutoSpeeds[2] = robotSpeeds.omegaRadiansPerSecond;
        setRobotSpeeds(new ChassisSpeeds(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond,
                -robotSpeeds.omegaRadiansPerSecond));
    }

    /// setting velocities of each module
    public void setModuleVelocities(Module module, ChassisSpeeds speeds) {
        double Vix = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * module.radius.getY();
        double Viy = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * module.radius.getX();

        double speed = Math.sqrt(Vix * Vix + Viy * Viy);
        double angle = Units.radiansToDegrees(Math.atan(Viy / Vix)) + (Vix < 0 ? 180 : 0);

        module.setSpeedAndAngle(speed, angle);
    }

    /// vision
    public void updatePoseEstimator() {
        /// updates the pose estimation our odometry to make sure when there is no vision, it uses data from the wheels.
        poseEstimator.update(getGyroAngle(), getModPositions());

        /// gets all the pose estimation from multiple limelight in the robot.
        ArrayList<LimeLightHelpers.PoseEstimate> poseEstimates = LimeVision.getPoseEstimates();

        /// adds each of those pose estimation to the pose estimator's vision measurement method
        /// and then internally, they calculate what is probablt the most accurate position based on multiple outputs.
        for (LimeLightHelpers.PoseEstimate currentEstimate: poseEstimates) {
            poseEstimator.addVisionMeasurement(currentEstimate.pose, currentEstimate.timestampSeconds);
        }

        states.position = getPose();
    }

    /// resets it
    /// done at the start of auto
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getGyroAngle(), getModPositions(), pose);
    }

    /// all the methods below are self explanatory and can be searched online for more detail
    /// Swerve: Zero To Hero or whatever the youtube series is has great detail on this code and would make you an expert
    /// highly recommended to watch.


    /// just gets the gyro angle from the gyro.
    public Rotation2d getGyroAngle() {
        return gyro.getAngle();
    }

    /// resets the gyro
    public void resetGyro() {
        gyro.zeroYaw();
    }

    public SwerveModulePosition[] getModPositions() {
        SwerveModulePosition[] modPositions = {
                new SwerveModulePosition(modules.get(0).getDriveDistance(), modules.get(0).getAngle()),
                new SwerveModulePosition(modules.get(1).getDriveDistance(), modules.get(1).getAngle()),
                new SwerveModulePosition(modules.get(2).getDriveDistance(), modules.get(2).getAngle()),
                new SwerveModulePosition(modules.get(3).getDriveDistance(), modules.get(3).getAngle())
        };
        return modPositions;
    }

    public void setSwerveModuleStates(SwerveModuleState[] moduleStates) {
        // Make sure all the target speeds are less than the max speed in constants
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates,
                DriverStation.isAutonomousEnabled() ? Constants.Auto.maxDriveSpeed : Constants.Drive.maxDriveSpeedMpS);

        for (int i = 0; i < moduleStates.length; i++) {
            SwerveModuleState state = moduleStates[i];
            modules.get(i).setSpeedAndAngle(state.speedMetersPerSecond, state.angle.getDegrees());
        }
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < moduleStates.length; i++) {
            Module ithModule = modules.get(i);
            moduleStates[i] = new SwerveModuleState(ithModule.getDriveSpeed(), ithModule.getAngle());
        }
        return moduleStates;
    }

    public void stop() {
        for (Module mod : modules) {
            mod.stop();
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getAncientPose() {
        return new Pose2d(states.odomPoses[0], states.odomPoses[1], new Rotation2d(states.odomPoses[2]));
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] moduleStates = getSwerveModuleStates();
        if (moduleStates != null) {
            return Constants.Drive.kinematics.toChassisSpeeds(moduleStates);
        }
        DriverStation.reportError("Swerve Module States were null!", true);
        return null;
    }

    public void log() {
        for (Module module : modules) {
            SmartDashboard.putData(module.moduleId, module);
        }
        SmartDashboard.putData("Drive", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // odom pose x and y
        builder.setSmartDashboardType("NetworkTableTree");
        builder.addDoubleProperty("Angle", () -> states.odomPoses[2], null);
        builder.addDoubleProperty("PoseX", () -> states.odomPoses[0], null);
        builder.addDoubleProperty("PoseY", () -> states.odomPoses[1], null);
        builder.addDoubleProperty("SpeedX", () -> getChassisSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("SpeedY", () -> getChassisSpeeds().vyMetersPerSecond, null);
        
    }
}
