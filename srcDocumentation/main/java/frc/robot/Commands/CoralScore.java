package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Coral.CoralElevator;
import frc.robot.Subsystems.Coral.CoralEndEffector;

/// This command goes to a certain elevator position (l2/l3) and scores it immediately.
/// this is different to ElevatorPosition as Elevator Position only puts it at a certain setpoint, it dosnt score it
/// this is useful in auto, not teleop because u want full control.

public class CoralScore extends Command {

    CoralElevator elevator;
    CoralEndEffector effector;
    Double setpoint;
    Timer timer = new Timer();
    

    public CoralScore(CoralElevator _elevator, CoralEndEffector _effector, Double _setpoint) {
        this.elevator = _elevator;
        this.effector = _effector;
        this.setpoint = _setpoint;
        addRequirements(this.elevator, this.effector);

    }

    @Override
    public void initialize() {
        /// uses the elevator subsystem to set it at the certain setpoint
        /// the setpoint are different for l2 and l3 because they are different heights.
        this.elevator.setElevatorPosition(setpoint);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {

        /// this checks the position of the elevator to make sure it is at the correct setpoint
        /// afterwards, it sets the effector to -1, which pushes the coral that is inside to the pole to be scored.
        if (elevator.getElevatorPosition() + 0.5 >= setpoint) {
            this.effector.setEffectorPercentOutput(-1);
        }
        SmartDashboard.putNumber("setpointElevator", setpoint);
    }

    @Override
    public boolean isFinished() {
        /// this has a timer because in auto you cannot yield for too long at one action, it is too risky.
        if (timer.hasElapsed(1)) return true;
        return this.effector.getTOFDistance() > Constants.Coral.outTakeTreshhold;
    }

    @Override
    public void end(boolean interupted) {
        /// makes sure everything stops and the elevator goes back to the intake position for another coral.
        this.elevator.setElevatorPosition(Constants.Coral.intakePos);
        this.effector.setEffectorPercentOutput(0);

    }

}
