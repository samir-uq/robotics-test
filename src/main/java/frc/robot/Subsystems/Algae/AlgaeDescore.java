package frc.robot.Subsystems.Algae;

import org.littletonrobotics.junction.Logger;

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

/// This subsystem spins the motor that rotates the arm, which helps us dealgify
/// look at Algae.java for details on how things work in a more general manner
/// This has setters and getters that set the motor speed and position and gets them as well per needed by commands.

public class AlgaeDescore implements IAlgaeDescoreIO, Subsystem {

    SparkMax descore;
    SparkMaxConfig descoreconfig;
    SparkClosedLoopController descorePID;
    RelativeEncoder descoreRelativeEncoder;

    AlgaeDescoreStatesAutoLogged algaeDescoreStates = new AlgaeDescoreStatesAutoLogged();

    public AlgaeDescore(int anglerMotorCANID) {
        descore = new SparkMax(anglerMotorCANID, MotorType.kBrushless);
        descoreconfig = new SparkMaxConfig();
        descorePID = descore.getClosedLoopController();
        descoreRelativeEncoder = descore.getEncoder();

        descoreconfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.Algae.descoreCurrentLimit);
        descoreconfig.encoder
                .positionConversionFactor(Constants.Algae.descoreRotToDeg)
                .velocityConversionFactor(Constants.Algae.descoreRotToDeg / 60);
        descoreconfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(0, 0.5)
                .pid(0.015, 0, 0)
                .positionWrappingInputRange(0, 360);

        descore.configure(descoreconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        updateStates(algaeDescoreStates);
        Logger.processInputs("algae", algaeDescoreStates);
    }

    @Override
    public AlgaeDescoreStatesAutoLogged updateStates(AlgaeDescoreStatesAutoLogged _algaeDescoreStates) {

        _algaeDescoreStates.OutputCurrent = descore.getOutputCurrent();
        _algaeDescoreStates.Temp = descore.getMotorTemperature();

        _algaeDescoreStates.algaeDescorePosition = getDescorePosition();
        _algaeDescoreStates.descoreSpeed = getDescoreSpeed();

        return _algaeDescoreStates;
    }

    public double getDescorePosition() {
        return descoreRelativeEncoder.getPosition();
    }

    public double getDescoreSpeed() {
        return descoreRelativeEncoder.getVelocity();
    }

    public void setDescorePercentOutput(double speed) {
        descore.set(speed);
    }

    public void setDescorePosition(double angle) {
        descorePID.setReference(angle, ControlType.kPosition);
    }

}
