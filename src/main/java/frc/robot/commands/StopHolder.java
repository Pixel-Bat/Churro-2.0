package frc.robot.commands;

import frc.robot.subsystems.NoteHolder;

import edu.wpi.first.wpilibj2.command.Command;

public class StopHolder extends Command {
  private final NoteHolder m_holder;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopHolder(NoteHolder holder) {
    m_holder = holder;
    
    addRequirements(holder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_holder.setHolderSpeed(0);
    this.cancel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_holder.setHolderSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

