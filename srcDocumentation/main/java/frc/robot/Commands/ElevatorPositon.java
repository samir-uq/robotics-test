package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;

/// this is an abstract of CoralScore.java
/// it just raises the elevator to a certain setpoint, not score it.

public class ElevatorPositon extends Command{

    CoralElevator elevator;
    CoralEndEffector effector;
    Double setpoint;


    public ElevatorPositon(CoralElevator _elevator, Double _setpoint){
        this.elevator = _elevator;
        this.setpoint = _setpoint;
        addRequirements(this.elevator);

    }

    @Override
    public void execute() {
        /// sets it to a certain setpoint.
        this.elevator.setElevatorPosition(setpoint);       
    }

    @Override
    public boolean isFinished(){
        /// just waits until the elevator position is around the setpoint
        /// basically yields until it is there
        return Math.abs(elevator.getElevatorPosition() - setpoint) < 0.5;

    }


    @Override
    public void end(boolean interupted){
    }
    
}
