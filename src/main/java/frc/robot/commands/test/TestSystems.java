package frc.robot.commands.test;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;


public class TestSystems extends SequentialCommandGroup{
    private final ShooterIntake m_armIntake;
    private final NoteHolder m_noteHolder;
    private final BumperIntake m_bumperIntake;
    private final Pivot m_pivot;


    public TestSystems(Pivot pivot, ShooterIntake shooter, NoteHolder noteHolder, BumperIntake bumperIntake) {
        this.m_armIntake = shooter;
        this.m_noteHolder = noteHolder;
        this.m_bumperIntake = bumperIntake;
        this.m_pivot = pivot;
        addRequirements(shooter, noteHolder, bumperIntake);

        addCommands(

            // Wait a bit before doing everything
            new WaitCommand(1),

            // Test the pivot
            new TestPivot(m_pivot),

            // Wait 2 seconds before moving on
            new WaitCommand(2.0),

            // run the shooter for 2 seconds, then stop, wait 0.5 seconds, then shoot it
            shooter.intake().withTimeout(2),
            new WaitCommand(0.5),
            shooter.shoot().withTimeout(2),

            // Wait 2 seconds before moving on
            new WaitCommand(2.0),

            // run the note holder for 2 seconds, then stop, wait 0.5 seconds, then run it in the opposite direction
            noteHolder.intake().withTimeout(2),
            new WaitCommand(0.5),
            noteHolder.shoot().withTimeout(2),

            new WaitCommand(2.0),

            // run the ground intake for 2 seconds, then stop, wait 0.5 seconds, then run it in the opposite direction
            bumperIntake.intake().withTimeout(2),
            new WaitCommand(0.5),
            bumperIntake.release().withTimeout(2)
        );
    }
}
