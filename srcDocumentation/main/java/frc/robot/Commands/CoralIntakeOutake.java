package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralEndEffector;
import frc.robot.Utilities.Controller;

/// another intake, this command is used by the operator to get the coral from the funnel and score it in a pole.

public class CoralIntakeOutake extends Command {

    CoralEndEffector effector;
    Controller joy;

    public boolean hasCoral;

    public CoralIntakeOutake(CoralEndEffector _effector, Controller _joy) {
        this.effector = _effector;
        this.joy = _joy;
        addRequirements(this.effector);

    }

    @Override
    public void initialize() {
        /// checks if we have a coral or not
        /// using this, we decide how we want to spin the motor and wait for coral to be seen by the sensor or wait for it to dissapear
        if (this.effector.getTOFDistance() <= Constants.Coral.outTakeTreshhold) {
            hasCoral = true;
        } else {
            hasCoral = false;
        }
    }

    @Override
    public void execute() {
        /// spinning the motor a certain way
        if (!hasCoral) {
            effector.setEffectorPercentOutput(-0.3);
        } else {
            effector.setEffectorPercentOutput(-1);
        }
    }

    @Override
    public boolean isFinished() {
        /// using the snensor to see if theres no more coran if we are doing outtake and inverse for the intake.
        return this.effector.getTOFDistance() < Constants.Coral.outTakeTreshhold && !hasCoral
                || this.effector.getTOFDistance() > Constants.Coral.outTakeTreshhold && hasCoral;

    }

    @Override
    public void end(boolean interupted) {
        /// stopping motors.
        this.effector.setEffectorPercentOutput(0);
    }

}
