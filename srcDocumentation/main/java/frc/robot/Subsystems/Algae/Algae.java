package frc.robot.Subsystems.Algae;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/// This code has changed severely to fit the L1 intake/outtake instead of algae
/// you can look at Cage.java / CoralElevator.java for details on basis of subsystem and stuff
/// it does the same thing, has getters and setters
/// multiple motors, tuned pids and other stuff that help the subsystem run

public class Algae implements IAlgaeIO, Subsystem {

    SparkMax angler;
    SparkMaxConfig anglerconfig;
    SparkClosedLoopController anglerPID;
    RelativeEncoder anglerRelativeEncoder;

    SparkFlex intake;
    SparkFlexConfig intakeconfig;
    SparkClosedLoopController intakePID;
    RelativeEncoder intakeRelativeEncoder;

    AlgaeStatesAutoLogged algaeStates = new AlgaeStatesAutoLogged();

    public Algae(int anglerMotorCANID, int intakeMotorCANID) {
        angler = new SparkMax(anglerMotorCANID, MotorType.kBrushless);
        anglerconfig = new SparkMaxConfig();
        anglerPID = angler.getClosedLoopController();
        anglerRelativeEncoder = angler.getEncoder();

        anglerconfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        // anglerconfig.softLimit
        //         .reverseSoftLimit(0)
        //         .reverseSoftLimitEnabled(true)
        //         .forwardSoftLimit(75)
        //         .forwardSoftLimitEnabled(true);
        anglerconfig.encoder
                .positionConversionFactor(Constants.Algae.rotToDeg)
                .velocityConversionFactor(Constants.Algae.rotToDeg / 60);
        anglerconfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .outputRange(-0.75, 0.75)
                .pid(0.2, 0.000001, 0)
                .maxMotion
                .maxVelocity(30000)
                .maxAcceleration(30000)
                .allowedClosedLoopError(0.7);
        

        angler.configure(anglerconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intake = new SparkFlex(intakeMotorCANID, MotorType.kBrushless);
        intakeconfig = new SparkFlexConfig();
        intakePID = intake.getClosedLoopController();
        intakeRelativeEncoder = intake.getEncoder();

        intakeconfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        intakeconfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(0.002, 0, 0, 0.0001);

        intake.configure(intakeconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
        updateStates(algaeStates);
        Logger.processInputs("algae", algaeStates);
        System.out.println(getAnglerPosition());
    }

    @Override
    public AlgaeStatesAutoLogged updateStates(AlgaeStatesAutoLogged _algaeStates) {

        _algaeStates.OutputCurrent = angler.getOutputCurrent();
        _algaeStates.Temp = angler.getMotorTemperature();

        _algaeStates.algaeAnglerPosition = getAnglerPosition();
        _algaeStates.anglerSpeed = getAnglerSpeed();

        _algaeStates.intakeSpeed = getIntakeSpeed();

        return _algaeStates;
    }

    public double getAnglerPosition() {
        return anglerRelativeEncoder.getPosition();
    }

    public double getAnglerSpeed() {
        return anglerRelativeEncoder.getVelocity();
    }

    public double getIntakeSpeed() {
        return intakeRelativeEncoder.getVelocity();
    }

    public double getIntakeCurrent() {
        return intake.getOutputCurrent();
    }

    public void setAnglerPercentOutput(double speed) {
        angler.set(speed);
    }

    public void setIntakePercentOutput(double speed) {
        intake.set(speed);
    }

    public void setIntakeSpeed(double intakeSpeed) {
        intakePID.setReference(intakeSpeed, ControlType.kVelocity);
    }

    public void setAnglerPosition(double angle) {
        anglerPID.setReference(angle, ControlType.kMAXMotionPositionControl);
    }

}
