// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.*;




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

  private final PivotCopy pivot = new PivotCopy();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandJoystick operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    setupDefaultCommands();
    configureBindings();
    setupAutoChoosers();
  }

  private void setupDefaultCommands(){
    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        // () -> -this.driveController.getLeftY(),
        // () -> this.driveController.getRightX()/1.35
        () -> -this.driveController.getLeftY() / 1.5,
        () -> this.driveController.getRightX()/1.85
      )
    );
  }

  private void configureBindings() {
    //operatorController.button(1).whileTrue(intake.shoot());
    operatorController.button(2).whileTrue(intake.intake());
    operatorController.button(3).whileTrue(intake.amp());
    operatorController.button(4).whileTrue(noteHolder.shoot());
    operatorController.button(6).whileTrue(noteHolder.intake());
    //driveController.y().whileTrue(noteHolder.shoot());

    driveController.button(1).whileTrue(pivot.customPos());
    // operatorController.button(1).whileTrue(pivot.intakePos());    
    // operatorController.button(2).whileTrue(pivot.shootPos());    
  }

  private void setupAutoChoosers(){ 
    new PathPlannerAuto("Example Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    Shuffleboard.getTab("Test Tab").add("Auto Mode", autoChooser);
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
