package frc.robot.System;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// extends SubsystemBase means it inherits all the properties of the SubsystemBase
// System is where you would have all the advanced logic for one singular item
public class Motor extends SubsystemBase {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private RelativeEncoder relativeEncoder;
  private SparkClosedLoopController motorPID;

  /**
   * initializes Motor subsystem
   * 
   * @param motorId id of the motor.
   * @param motorType the type of the CanSpark motor
   */
  public Motor(int motorId, MotorType motorType) {
    motor = new SparkMax(motorId, motorType);
    motorConfig = new SparkMaxConfig();
    motorPID = motor.getClosedLoopController();
    relativeEncoder = motor.getEncoder();

    motorConfig
      .inverted(false) // if the motor spins in inverted or normal way
      .idleMode(IdleMode.kBrake) // wether the motor is taking a break at its position or just letting stuff be
      .smartCurrentLimit(30); // current limit of the motor

    // sets the soft limit of the motor. which just means like the maximum and minimum based on the software
    motorConfig.softLimit
      .forwardSoftLimit(23) // a soft limit on the highest the position of this motor can be
      .forwardSoftLimitEnabled(true) // enables the forward soft limit
      .reverseSoftLimit(-0.5) // minimum position this motor can be
      .reverseSoftLimitEnabled(true); // enables this.

    // if you ever dont know what any of these methods mean, right clikc on the method and press go to declaration
    motorConfig.encoder
      .positionConversionFactor(0.23) // conversion factor of the position to multiply by the motors natove
      .velocityConversionFactor(0.23 / 60); // conversion factor of the velocity

    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // the sensor used to operate the pid
      .pid(20,0,0.2) // the actual PID
      .outputRange(-1, 1) // so the minumum and maximum the speed of the motor can be from range [-1, 1]
      .maxMotion // max motion is like advanced Can Spark motor handling
      .maxVelocity(1e10)
      .maxAcceleration(12e10)
      .allowedClosedLoopError(0.7); // this means the pid keeps on retrying until the error is less than 0.7

    resetEncoder();
    // the pid numbers, P, I, D, need to be configured until they are properly tuned
    // this year we are going to look at LoggedTunableNumbers which allows us to tune this while the robot is on.
  }

  public Motor(int motorId) {
    this(motorId, MotorType.kBrushless);
  }

  /**
   * sets the speed of the Motor using the motor PID
   * 
   * @param speed The speed the motor should be
   */
  public void setSpeed(double speed) {
      motorPID.setReference(speed, ControlType.kMAXMotionVelocityControl);

      // this is different from the typical
      // motor.setSpeed(x);
      // mainly due to the fact that it is doing it using the PID and not the actual motor
      // this means that the pid has control of the real speed, which is a good thing, as long as its tuned
  }

  /**
   * Sets the position of the motor using the motor pid in a certain position
   * 
   * @param pos the position the motor needs to be in
   */
  public void setPosition(double pos) {
    motorPID.setReference(pos, ControlType.kMAXMotionPositionControl);
  }


  /**
   * Sets the position of the encoder to 0 so now the default postion would be whatever it is right now
   */
  public void resetEncoder() {
    relativeEncoder.setPosition(0);
  }

  // within subsystem base, it has methods such as periodic
  // periodic is called once per scheduled run, which can also be said as "per frame"
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // this would be called whenever Simulation is selected under driver station.
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
