package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.System.Motor;

public class RunMotor extends Command {
  private final Motor m_subsystem;

  // ALL_CAPS are meant to be for constants
  private final double MOTOR_SPEED = 0.4;

  /**
   * Creates a new RunMotor command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunMotor(Motor subsystem) {
    m_subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  // can be called multiple time as the same command object is used
  @Override
  public void initialize() {
    m_subsystem.setSpeed(MOTOR_SPEED);
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
    m_subsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
