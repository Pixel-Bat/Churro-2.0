package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.IntakeID;
import frc.robot.Constants.ShooterConstants;



public class ShooterIntake extends SubsystemBase{
    

    private final CANSparkMax m_shooterMotor = new CANSparkMax(IntakeID.intakeMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    

    public ShooterIntake() {
        
    }

    public Command intake() {
        return runEnd(() -> {
            m_shooterMotor.set(ShooterConstants.shooterSpeedIn);
        }, () -> {
            m_shooterMotor.set(0);
        });
    }

    public Command shoot() {
        return runEnd(() -> {
            m_shooterMotor.set(ShooterConstants.shooterSpeedOut);
        }, () -> {
            m_shooterMotor.set(0);
        });
    }

    public Command amp() {
        return runEnd(() -> {
            m_shooterMotor.set(ShooterConstants.ampSpeedOut);
        }, () -> {
            m_shooterMotor.set(0);
        });
    }
}
