package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Algae.AlgaeDescore;

/// Descore algae command
/// do acknowledge that our dealgify is just a stick that spins

public class DescoreAlgae extends Command {
    AlgaeDescore descore;
    double setpoint;

    public DescoreAlgae(AlgaeDescore _descore, double _setpoint) {
        descore = _descore;
        setpoint = _setpoint;
        addRequirements(descore);

    }

    @Override
    public void execute() {
        /// this is a stick that spins to a certain setpoint
        /// look more into the subsystem to see how this method functions.
        descore.setDescorePercentOutput(setpoint);
    }

 
    @Override
    public void end(boolean interupted) {
        // stops the motor
       descore.setDescorePercentOutput(0);

    }


    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(descore);
    }

}