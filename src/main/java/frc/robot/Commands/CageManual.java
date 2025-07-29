package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Cage.Cage;
import frc.robot.Utilities.Controller;

/// Command used to control the cage

public class CageManual extends Command {
    Cage cage;
    Controller joy;
    double position;


    public CageManual(Cage _cage, Controller _joy) {
        cage = _cage;
        joy = _joy;
        addRequirements(cage);
        /// same requirements stuff
        /// also has the joystick here
    }

    @Override
    public void initialize() {
    }

    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        /// basically, if you are holding the left trigger, it pulls the cage up
        /// if you are holding the right trigger, it pulls the cage down
        
        /// you can see how it uses the cage subsystem with absolute position and other things to make sure it does as it behaves.
        /// more of these methods can be found on the cage subsystem.
        if (joy.getLeftTriggerAxis() > 0.2 && cage.getCageAbsPosition() > 90) {
            cage.setCagePercentOutput(1);
            position = cage.getCagePosition();
        }

        else if (joy.getRightTriggerAxis() > 0.2 && cage.getCageAbsPosition() < 131) {
            cage.setCagePercentOutput(-1);
            position = cage.getCagePosition();
        }

        else if (joy.getRightTriggerAxis() > 0.2 && cage.getCageAbsPosition() > 131) {
            cage.setCagePosition(position);
        }

        else if (joy.getLeftTriggerAxis() > 0.2 && cage.getCageAbsPosition() < 90) {
            cage.setCagePosition(position);
        }

        else {
            cage.setCagePosition(position);
        }
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(cage);
    }

}