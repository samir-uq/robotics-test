package frc.robot.Subsystems.Cage;

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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/// Main Cage Subsystem

public class Cage implements ICageIO, Subsystem {

    /// different motors, encoders
    /// pid and config for the motors as well
    SparkMax cage;
    SparkMaxConfig cageconfig;
    SparkClosedLoopController cagePID;
    RelativeEncoder cageRelativeEncoder;
    DutyCycleEncoder absencoder;

    CageStatesAutoLogged cageStates = new CageStatesAutoLogged();

    public Cage(int cagerMotorCANID) {
        /// setting up the motor
        /// SparkMax library describes all of this.
        /// it is all about configuration and pid to make a motor work properly.
        cage = new SparkMax(cagerMotorCANID, MotorType.kBrushless);
        cageconfig = new SparkMaxConfig();
        cagePID = cage.getClosedLoopController();
        cageRelativeEncoder = cage.getEncoder();
        absencoder = new DutyCycleEncoder(Constants.Cage.absEncoderChannel);

        absencoder.setInverted(true);
        cageconfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Constants.Cage.cageCurrentLimit);
        cageconfig.encoder
                .positionConversionFactor(Constants.Cage.rotToDeg)
                .velocityConversionFactor(Constants.Cage.rotToDeg / 60);
        cageconfig.closedLoop
                .outputRange(-0.7, 0.7)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0, 0);

        cage.configure(cageconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /// updates the state of the cage every "frame" and logs it.
    @Override
    public void periodic() {
        updateStates(cageStates);
        Logger.processInputs("cage", cageStates);
    }

    /// method that updates the states
    @Override
    public CageStatesAutoLogged updateStates(CageStatesAutoLogged _cageStates) {
        /// these are just values that are useful to debug certain issues
        /// they are done using AutoLog, more on this on ICageIO.java
        _cageStates.OutputCurrent = cage.getOutputCurrent();
        _cageStates.Temp = cage.getMotorTemperature();

        _cageStates.cagePosition = getCagePosition();
        _cageStates.cageSpeed = getCageSpeed();

        return _cageStates;
    }

    /// all of the methods from here to bottom are wrappers to getting information from the encoders of a motor
    /// it is very important to know how motors work. once you know that, the robot is basically coded.
    /// PIDS, MOTORS are the main things.
    /// also these are just getter and setters

    public double getCagePosition() {
        return cageRelativeEncoder.getPosition();
    }

    public double getCageAbsPosition() {
        return absencoder.get()* 360;
    }

    public double getCageSpeed() {
        return cageRelativeEncoder.getVelocity();
    }

    public void setCagePercentOutput(double speed) {
        cage.set(speed);
    }

    public void setCagePosition(double angle) {
        /// the control type is a feature of SparkMax, they help us set the type the motor is in
        cagePID.setReference(angle, ControlType.kPosition);
    }

}
