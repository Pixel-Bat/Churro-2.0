package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BumperIntake;
import frc.robot.subsystems.NoteHolder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.ShooterIntake;

public class GroundIntakeToggle extends Command{
    private final Pivot m_pivot;
    private final ShooterIntake m_armIntake;
    private final NoteHolder m_noteHolder;
    private final BumperIntake m_bumperIntake;

    private boolean toggle = false;

    public GroundIntakeToggle (Pivot pivot, ShooterIntake armIntake, NoteHolder noteHolder, BumperIntake bumperIntake) {
        this.m_pivot = pivot;
        this.m_armIntake = armIntake;
        this.m_noteHolder = noteHolder;
        this.m_bumperIntake = bumperIntake;
        addRequirements(pivot, armIntake, noteHolder, bumperIntake);
    }

    @Override
    public void initialize(){
        if(toggle) {
            toggle = false;
            new GroundIntakeStart(this.m_pivot, this.m_armIntake, this.m_noteHolder, this.m_bumperIntake);
        } else if(!toggle) {
            toggle = true;
            new GroundIntakeEnd(this.m_armIntake, this.m_noteHolder, this.m_bumperIntake);
        }
    }
    
}


