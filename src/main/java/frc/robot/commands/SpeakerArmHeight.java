// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.Constants.PivotConstants;
// import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;


public class SpeakerArmHeight extends Command {
  private final Pivot m_pivot;

  public SpeakerArmHeight(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    m_pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.autoArmHeight();

    if(m_pivot.leftPivotController.atSetpoint())
      end(isScheduled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}