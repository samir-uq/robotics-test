package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Algae.Algae;

/// Command for scoring the Coral L1

public class CoralL1score extends Command {

    Algae algae;

    public CoralL1score(Algae _algae) {
        this.algae = _algae;
        addRequirements(this.algae);
    }

    @Override
    public void initialize() {
        this.algae.setAnglerPosition(Constants.Algae.coralScorePos);
        /// sets the coral intake arm to a certain setpoint
    }

    @Override
    public void execute() {
        /// this waits to see if the position of the arm is at a certain angle so it could drop it and it would go into the l1
        /// if not for this if statement, it would immediately shoot it out, which would not put it in the l1 because the default position is upright.
        if(algae.getAnglerPosition() >= 10){
         this.algae.setIntakePercentOutput(0.5);
        }
        
    }


    @Override
    public void end(boolean interupted) {
        /// makes every motor for this to  0
        this.algae.setIntakePercentOutput(0);
        this.algae.setAnglerPosition(Constants.Algae.homePos);
    }

}
