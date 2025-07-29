package frc.robot.Subsystems.Coral;

import org.littletonrobotics.junction.Logger;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/// Refer to CoralElevator.java
/// This subsystem just spins the motor that is connected to the wheel. it functions as any other subsystem and is really basic
/// it also sets up the time of flight sensor, which helps us detect if there is a coral or not

public class CoralEndEffector implements Subsystem, ICoralEndEffectorIO {
    SparkMax effector;
    SparkMaxConfig effectorConfig;
    SparkClosedLoopController effectorPID;
    RelativeEncoder relativeEncoder;
    TimeOfFlight TOFSensor;
    TimeOfFlight AutoTOFSensor;

    EffectorStatesAutoLogged effectorStates = new EffectorStatesAutoLogged();

    public CoralEndEffector(int EffectorMotorCANID) {
        effector = new SparkMax(EffectorMotorCANID, MotorType.kBrushless);
        effectorConfig = new SparkMaxConfig();
        effectorPID = effector.getClosedLoopController();
        relativeEncoder = effector.getEncoder();

        effectorConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.Coral.endEffectorCurrentLimit);
        effectorConfig.encoder
                .positionConversionFactor(Constants.Coral.rotToDeg)
                .velocityConversionFactor(Constants.Coral.rotToDeg / 60);
        effectorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.01, 0.0, 0)
                .outputRange(-1, 1);

        effector.configure(effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        TOFSensor = new TimeOfFlight(21);
        /// you can see the sensor ranging mode, this can be found on their documentation and just plugged in.
        TOFSensor.setRangingMode(RangingMode.Short, 24);

    }

    @Override
    public void periodic() {
        updateStates(effectorStates);
        Logger.processInputs("effector", effectorStates);

    }

    @Override
    public EffectorStatesAutoLogged updateStates(EffectorStatesAutoLogged _effectorStates) {

        _effectorStates.OutputCurrent = effector.getOutputCurrent();
        _effectorStates.Temp = effector.getMotorTemperature();
        _effectorStates.effectorSpeed = geEffectorSpeed();

        return _effectorStates;
    }

    public double geEffectorSpeed() {
        return relativeEncoder.getVelocity();
    }

    public void setEffectorPercentOutput(double speed) {
        effector.set(speed);
    }

    public void resetEncoder() {
        relativeEncoder.setPosition(0);
    }

    public void setEffectorSpeed(double spd) {
        effectorPID.setReference(spd, ControlType.kVelocity);
    }

    public double getTOFDistance() {
        return TOFSensor.getRange();
    }

}