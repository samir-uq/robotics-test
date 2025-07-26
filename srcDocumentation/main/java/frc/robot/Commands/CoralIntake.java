package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;
import frc.robot.Utilities.Controller;

/// Command that is just to intake the coral from the funnel.

public class CoralIntake extends Command{

    CoralElevator elevator;
    CoralEndEffector effector;
    Controller joy;


    public CoralIntake(CoralElevator _elevator, CoralEndEffector _effector, Controller _joy){
        this.elevator = _elevator;
        this.effector = _effector;
        this.joy = _joy;
        addRequirements(this.elevator, this.effector);
    }

    @Override
    public void initialize(){
        this.elevator.setElevatorPosition(Constants.Coral.intakePos);
       /// sets the elevator position to the intake so the coral can actually slide through.
    }

    @Override
    public void execute() {
        this.effector.setEffectorPercentOutput(-0.3);
    }
    @Override
    public boolean isFinished(){
        /// this uses a distance sensor to see if there is a coral in between the sensor and the other surface or not.
        /// this allows us to not wait on timing anything and making sure we dont spin it too much.
        return this.effector.getTOFDistance() < Constants.Coral.outTakeTreshhold;
    }

    @Override
    public void end(boolean interupted){
   
        this.elevator.setElevatorPosition(Constants.Coral.intakePos);
        this.effector.setEffectorPercentOutput(0);
        /// just stops all the motors.
    }

    
}
