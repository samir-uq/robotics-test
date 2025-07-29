package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Algae.Algae;

/// Intaking Algae Code


public class AlgaeIntake extends Command {

    Algae algae;
    private final Timer timer = new Timer();

    public AlgaeIntake(Algae _algae) {
        this.algae = _algae;

        /// add requirement just makes sure that a subsystem is required to make this command functional, tying the command to the subsystem
        addRequirements(this.algae);
    }

    @Override
    public void initialize() {
        /// when the button is pressed, initialize is called
        /// this uses the algae subsystem to set the angle position of the motor to a preset position (the intake position)
        this.algae.setAnglerPosition(Constants.Algae.algaeIntakePos);

        /// we restart the timer to run the time again
        /// this was because at one point we ran this based on the timer not based on holding
        /// now it is holding and this is obsolete.
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        /// every "frame" we make sure that the intake (which is the green wheels) have an output so when the algae comes in contact, it actually gets the ball
        this.algae.setIntakePercentOutput(0.5);
    }


    @Override
    public void end(boolean interupted) {
        /// at the end of the command, we make the angler position back to the default one
        /// and also make sure the green wheels stop spinning.
        this.algae.setAnglerPosition(Constants.Algae.homePos);
        this.algae.setIntakePercentOutput(0);
        timer.stop();
    }

}
