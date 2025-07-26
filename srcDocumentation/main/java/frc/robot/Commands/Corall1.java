package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Algae.Algae;
import frc.robot.Utilities.Controller;

/// Command for Coral L1 Initialize
/// this code is really bad (everything in coral l1) because it was done during pit time in Worlds

public class Corall1 extends Command {
    Algae algae;
    Controller joy;
    double position;


    public Corall1(Algae _algae, Controller _joy) {
        algae = _algae;
        joy = _joy;
        addRequirements(algae);

    }

    @Override
    public void initialize() {
        /// just sets it at 0
        algae.setAnglerPosition(0);
    }

    public void end(boolean interrupted) {
    }

    @Override
    public void execute() {
        
    }

}