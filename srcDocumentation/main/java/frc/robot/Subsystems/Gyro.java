package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/// This is the gyro we use for our drivetrain

public class Gyro {
    static Pigeon2 pidgey = new Pigeon2(50);

    // docs
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/Pigeon2.html

    public Gyro(int CANID) {
        zeroYaw();
        pidgey.reset();
    }

    public Rotation2d getAngle() {
        /// just gets the Rotation2d angle off the gyro
        return pidgey.getRotation2d();
    }

    public void testPigeon() {
        /// to test and put it in our dashboard
        SmartDashboard.putNumber("Pigeon 2.0 Test Angle", getAngle().getDegrees());
    }

    public static double getYaw() {
        /// returns the yaw of the robot
        StatusSignal<Angle> angleSignal = pidgey.getYaw();
        return angleSignal.getValueAsDouble();
    }

    public static double getRelativeYaw(double absoluteYaw) {
        /// returns the yaw of the robot - whatever to give the "absoluteYaw"
        return getYaw() - absoluteYaw;
    }

    public void zeroYaw() {
        /// zeroes the yaw, as mentioned in DriveManual.java
        pidgey.setYaw(0);
    }
}
