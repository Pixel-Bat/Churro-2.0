// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.GroundIntakeToggle;
// import frc.robot.commands.SpeakerArmHeight;
// import frc.robot.commands.StopHolder;
// import frc.robot.commands.StopShooter;
// import frc.robot.commands.GroundIntakeStart;
// import frc.robot.commands.GroundIntakeEnd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.test.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final ShooterIntake shooter = new ShooterIntake();

  private final NoteHolder noteHolder = new NoteHolder();

  private final BumperIntake bumperIntake = new BumperIntake();

  private final BumperIntake groundintake = new BumperIntake();

  private final Pivot pivot = new Pivot();

  private final PowerHub powerHub = new PowerHub();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS5Controller driveController = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);
  public static final CommandJoystick operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);
  private final SendableChooser<Command> autoChooser; // Default auto will be `Commands.none()`

  
  private final SendableChooser<Command> testChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    defineAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    setupDefaultCommands();
    configureBindings();
    setupAutoChoosers();
    setupTestChoosers();
  }

  public void defineAutoCommands() {
    // Setting up NamedCommands from the PathPlanner Auto
    NamedCommands.registerCommand("setIntakePos", pivot.intakePos());
    NamedCommands.registerCommand("setShootPos", pivot.speakerPos());
    NamedCommands.registerCommand("startShooter", shooter.shootNoEnd());
    NamedCommands.registerCommand("shootNote", noteHolder.shootNoEnd());
    NamedCommands.registerCommand("intakeNote", shooter.intakeNoEnd());
    NamedCommands.registerCommand("stopShooter", new StopShooter(shooter));
    NamedCommands.registerCommand("stopHolder", new StopHolder(noteHolder));
    NamedCommands.registerCommand("groundIntakeStart", groundintake.intake());
    NamedCommands.registerCommand("groundIntakeStop", groundintake.stop());
  }

  private void setupDefaultCommands(){
    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        () -> (Math.pow(-this.driveController.getLeftY(), 3))*0.4,

        // () -> this.driveController.getRightX()/1.05 // for PS5
        () -> this.driveController.getRawAxis(4)/2.5 // for logitech
      )
    );

    powerHub.setDefaultCommand(powerHub.retract());
  }
  private void configureBindings() {
    operatorController.button(1).whileTrue(shooter.shoot());
    operatorController.button(2).whileTrue(shooter.intake());
    operatorController.button(2).whileTrue(bumperIntake.intake());
    operatorController.button(3).whileTrue(shooter.amp());
    operatorController.button(4).whileTrue(noteHolder.shoot());
    operatorController.button(6).whileTrue(noteHolder.intake());
    operatorController.button(9).whileTrue(bumperIntake.release());

    operatorController.button(11).whileTrue(pivot.intakePos());
    operatorController.button(8).whileTrue(pivot.ampPos());
    operatorController.button(12).whileTrue(pivot.speakerPos());
    operatorController.button(10).whileTrue(pivot.sourcePos());
    operatorController.button(9).whileTrue(pivot.flatPos());
    operatorController.button(7).whileTrue(pivot.ampPos2());
    

    // driveController.button(6).whileTrue(bumperIntake.intake()); 
    // driveController.button(6).whileTrue(shooter.intake());
    
    // driveController.button(5).whileTrue(pivot.intakePos()); 
    //New code

    //operatorController.button(7).whileTrue(groundintake.groundIn());
    
    //driveController.y().whileTrue(noteHolder.shoot());

    //operatorController.button(1).whileTrue(pivot.customPos(operatorController.getY()));
    //operatorController.button(1).whileTrue(pivot.intakePos());    
  }

  private void setupAutoChoosers(){ 
    new PathPlannerAuto("Example Auto");
    new PathPlannerAuto("2024 ONT McMaster Auto 1");
    new PathPlannerAuto("2024 ONT McMaster Auto 2 (Test)");
    new PathPlannerAuto("NO MOVEMENT TESTING ONLY");
    new PathPlannerAuto("1 Note Bottom");
    SmartDashboard.putData("Auto Mode", autoChooser);
    Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).add("Auto Mode", autoChooser);
  }

  private void setupTestChoosers(){

    testChooser.addOption("All Systems", new TestSystems(pivot,shooter, noteHolder, groundintake));
    testChooser.addOption("Pivot", new TestPivot(pivot));

    Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).add("Test Mode", testChooser);
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

  public Command getTestCommand(){
    return testChooser.getSelected();
  }
}
