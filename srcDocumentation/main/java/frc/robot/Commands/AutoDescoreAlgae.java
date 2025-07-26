package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.Algae.AlgaeDescore;


/// This command was created to dealgify in auto
/// never tested.

public class AutoDescoreAlgae extends Command {
    AlgaeDescore descore;
    double setpoint;
    private final Timer timer = new Timer();

    public AutoDescoreAlgae(AlgaeDescore _descore, double _setpoint) {
        descore = _descore;
        setpoint = _setpoint;
        addRequirements(descore);
        
        /// same thing with add requirements
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        //// uses the descore subsystem to set it at a certain setpoint (so it can rotate)
        descore.setDescorePercentOutput(setpoint);
    }

    @Override
    public boolean isFinished() {
        /// this is timed so after every execute, it checks if 1 second has passed since the timer , it ends this command by returning true.
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interupted) {

        /// makes sure that the motors stop spinning
        descore.setDescorePercentOutput(0);
        timer.stop();
    }

    @Override
    public Set<Subsystem> getRequirements() {
        /// get requirements and add requirements are basically the same thing, u can look into documentation for that.
        return Set.of(descore);
    }

}