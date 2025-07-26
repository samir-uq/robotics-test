package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Algae.Algae;

/// The intake for the Coral L1
/// Also you can see most of this just has algae subsystem.
/// This was because it was replaced.

public class CoralL1intake extends Command {

    Algae algae;

    public CoralL1intake(Algae _algae) {
        this.algae = _algae;
        addRequirements(this.algae);
    }

    @Override
    public void initialize() {
        /// just sets the position to be in a setpoint to where a l1 can be intaken using the wheels.
        this.algae.setAnglerPosition(Constants.Algae.coralIntakePos);
  
    }

    @Override
    public void execute() {
        this.algae.setIntakePercentOutput(-0.24);
    }


    @Override
    public void end(boolean interupted) {
        /// puts the thing at default position, still has an intake motor spinning because the coral would fall out otherwise.
        /// this gets stopped in coral l1 score so it is not really a bug but it is not ideal either.
        this.algae.setAnglerPosition(Constants.Algae.homePos);
        this.algae.setIntakePercentOutput(-0.20);
    }

}
