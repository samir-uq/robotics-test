package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.System.Motor;

public class SetMotor extends Command {
  private final Motor m_subsystem;

  // ALL_CAPS are meant to be for constants
  private final double position;

  /**
   * Creates a new RunMotor command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetMotor(Motor subsystem, double position) {
    this.position = position;
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  // can be called multiple time as the same command object is used
  @Override
  public void initialize() {
    // at the start of this command, it puts the position of the motor to whatever the constructor was supplied with
    m_subsystem.setPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // generally youd have things here for calculations, but right now none of that is needed for this Command
    // the only use of this command is to make it run while the command is running
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  // in this case, immedietly
  @Override
  public boolean isFinished() {
    return true;
  }
}
