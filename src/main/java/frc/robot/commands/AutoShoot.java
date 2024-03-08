// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;


public class AutoShoot extends Command {
  private final Drivetrain m_drivebase;

  private final DoubleSupplier m_speedSupplier;
  private final DoubleSupplier m_rotationSupplier;
  public AutoShoot(Drivetrain drivebase, DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
    m_drivebase = drivebase;
    m_speedSupplier = speedSupplier;
    m_rotationSupplier = rotationSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebase.Drive(
      m_speedSupplier.getAsDouble(), 
      m_rotationSupplier.getAsDouble()
    );
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