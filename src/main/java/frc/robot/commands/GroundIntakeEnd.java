package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;


public class GroundIntakeEnd extends SequentialCommandGroup{
    private final ShooterIntake m_armIntake;
    private final NoteHolder m_noteHolder;
    private final BumperIntake m_bumperIntake;


    public GroundIntakeEnd(ShooterIntake armIntake, NoteHolder noteHolder, BumperIntake bumperIntake) {
        this.m_armIntake = armIntake;
        this.m_noteHolder = noteHolder;
        this.m_bumperIntake = bumperIntake;
        addRequirements(armIntake, noteHolder, bumperIntake);

        addCommands(
            new ParallelCommandGroup(
                bumperIntake.intake(),
                armIntake.intake(),
                noteHolder.intake()
            ) 
        );
    }
}
