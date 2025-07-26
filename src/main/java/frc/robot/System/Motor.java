package frc.robot.System;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// extends SubsystemBase means it inherits all the properties of the SubsystemBase
public class Motor extends SubsystemBase {
  private SparkMax motor;

  public Motor(int motorId, MotorType motorType) {
    motor = new SparkMax(motorId, motorType);
  }

  public Motor(int motorId) {
    this(motorId, MotorType.kBrushless);
  }

  /**
   * 
   * @param speed The speed the motor should be
   */
  public void setSpeed(double speed) {
    motor.set(speed);
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
