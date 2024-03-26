// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final ShooterIntake intake = new ShooterIntake();

  private final NoteHolder noteHolder = new NoteHolder();

  private final Pivot pivot = new Pivot();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driveController = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  public static final CommandJoystick operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);
  private final SendableChooser<Command> autoChooser; // Default auto will be `Commands.none()`

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    defineAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    setupDefaultCommands();
    configureBindings();
    setupAutoChoosers();
  }

  public void defineAutoCommands() {
    // Setting up NamedCommands from the PathPlanner Auto
    NamedCommands.registerCommand("setArmHeight", pivot.sourcePos());
    NamedCommands.registerCommand("startShooter", new ShooterShoot(intake));
    NamedCommands.registerCommand("stopShooter", new StopShooter(intake));
    NamedCommands.registerCommand("stopHolder", new StopHolder(noteHolder));
    NamedCommands.registerCommand("shootNote", new HolderShoot(noteHolder));
  }

  private void setupDefaultCommands(){
    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        // () -> 1.2 * Math.pow(-this.driveController.getLeftY(), 3),
        // () -> Math.pow(this.driveController.getRightX(), 3)
        () -> Math.pow(-this.driveController.getLeftY() / 1, 3),
        () -> Math.pow(this.driveController.getRightX()/1.2, 3)
      )
    );
  }

  private void configureBindings() {
    operatorController.button(1).whileTrue(intake.shoot());
    operatorController.button(2).whileTrue(intake.intake());
    operatorController.button(3).whileTrue(intake.amp());
    operatorController.button(4).whileTrue(noteHolder.shoot());
    operatorController.button(6).whileTrue(noteHolder.intake());

    // operatorController.button(8).onTrue(new PivotSpeaker(pivot));
    // operatorController.button(10).onTrue(new PivotAmp(pivot));
    // operatorController.button(12).onTrue(new PivotSource(pivot));
    // operatorController.button(11).onTrue(new PivotIntake(pivot));   

    operatorController.button(8).onTrue(pivot.speakerPos());
    operatorController.button(10).onTrue(pivot.ampPos());
    operatorController.button(12).onTrue(pivot.sourcePos());
    operatorController.button(11).onTrue(pivot.intakePos());


  }

  private void setupAutoChoosers(){ 
    new PathPlannerAuto("Example Auto");
    new PathPlannerAuto("2024 ONT McMaster Auto 1");
    new PathPlannerAuto("2024 ONT McMaster Auto 2 (Test)");
    SmartDashboard.putData("Auto Mode", autoChooser);
    Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).add("Auto Mode", autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
