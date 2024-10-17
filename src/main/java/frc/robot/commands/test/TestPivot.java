// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.PivotConstants;


public class TestPivot extends Command {
  private final Pivot m_pivot;

  private boolean checkedSpeaker;
  private boolean checkedIntake;


  public TestPivot(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    m_pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!checkedSpeaker){
      m_pivot.setPivotAngle(PivotConstants.positions.speakerHeight);

      if(m_pivot.encoderInDegrees() <= PivotConstants.positions.speakerHeight + 5 || m_pivot.encoderInDegrees() >= PivotConstants.positions.speakerHeight - 5){
        checkedSpeaker = true;
      }
    }

    if(!checkedIntake){
      m_pivot.setPivotAngle(PivotConstants.positions.intakeHeight);

      if(m_pivot.encoderInDegrees() <= PivotConstants.positions.intakeHeight + 5 || m_pivot.encoderInDegrees() >= PivotConstants.positions.intakeHeight - 5){
        checkedIntake = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setPivotAngle(PivotConstants.positions.speakerHeight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    if(checkedIntake && checkedSpeaker){
      return true;
    }

    return false;
  }
}