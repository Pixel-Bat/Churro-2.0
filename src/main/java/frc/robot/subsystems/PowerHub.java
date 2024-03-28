package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerHub extends SubsystemBase{
    private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

    public PowerHub() {
        m_pdh.setSwitchableChannel(true);
    }

    public Command retract() {
        return runEnd(() -> {
            m_pdh.setSwitchableChannel(true);
        }, () -> {
            m_pdh.setSwitchableChannel(false);
        });
    }
}