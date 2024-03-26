package frc.robot.commands;

import frc.robot.subsystems.ShooterIntake;

import edu.wpi.first.wpilibj2.command.Command;

public class StopShooter extends Command {
  private final ShooterIntake m_shooter;

  private boolean stop = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StopShooter(ShooterIntake shooter) {
    m_shooter = shooter;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterSpeed(0);
    stop = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(stop=true) {
        return true;
    }
    return false;
  }
}

