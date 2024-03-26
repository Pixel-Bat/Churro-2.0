package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.Constants.PivotConstants.positions;

public class PivotSource extends Command{

    private Pivot m_Pivot;

    public PivotSource(Pivot pivot) {
        this.m_Pivot = pivot;
        addRequirements(pivot);
    }

    @Override
    public void initialize(){
        m_Pivot.setPivotAngle(positions.sourceHeight);
    }
}
