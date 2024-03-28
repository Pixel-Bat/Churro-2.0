package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;


public class GroundIntakeStart extends SequentialCommandGroup{
    private final Pivot m_pivot;
    private final ShooterIntake m_armIntake;
    private final NoteHolder m_noteHolder;
    private final BumperIntake m_bumperIntake;


    public GroundIntakeStart(Pivot pivot, ShooterIntake armIntake, NoteHolder noteHolder, BumperIntake bumperIntake) {
    // public GroundIntakeGroup(Pivot pivot) {
        this.m_pivot = pivot;
        this.m_armIntake = armIntake;
        this.m_noteHolder = noteHolder;
        this.m_bumperIntake = bumperIntake;
        addRequirements(pivot, armIntake, noteHolder, bumperIntake);

        addCommands(
            new ParallelRaceGroup(
                pivot.intakePos(),
                new WaitCommand(0.1)
            ), 
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                new ParallelRaceGroup(
                    bumperIntake.intakeNoEnd(),
                    new WaitCommand(0.1)
                ),
                new ParallelRaceGroup(
                    armIntake.intakeNoEnd(),
                    new WaitCommand(0.1)
                ),
                new ParallelRaceGroup(
                    noteHolder.intakeNoEnd(),
                    new WaitCommand(0.1)
                )
            ) 
        );
    }
}
