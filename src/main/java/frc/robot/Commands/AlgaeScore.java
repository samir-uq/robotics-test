package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Algae.Algae;

/// this is basically the same as AlgaeIntake.java but instead the position and output percent are different
/// basically inverse of AlgaeIntake.java

public class AlgaeScore extends Command {

    Algae algae;
    private final Timer timer = new Timer();

    public AlgaeScore(Algae _algae) {
        this.algae = _algae;
        addRequirements(this.algae);
    }

    @Override
    public void initialize() {
        this.algae.setAnglerPosition(Constants.Algae.algaeScorePos);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        this.algae.setIntakePercentOutput(-0.5);
    }

    // @Override
    // public boolean isFinished() {
    //     return timer.hasElapsed(3);
    // }

    @Override
    public void end(boolean interupted) {
        this.algae.setIntakePercentOutput(0);
        this.algae.setAnglerPosition(Constants.Algae.homePos);
        timer.stop();
    }

}
