package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
   
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.BumperIntakeID;
import frc.robot.Constants.BumperConstants;



public class BumperIntake extends SubsystemBase{
    
    private final TalonSRX m_bumperMotor = new TalonSRX(BumperIntakeID.bumperIntakeMotorCANID); //Different motor

    public void setHolderSpeed(double speed) {
        m_bumperMotor.set(ControlMode.PercentOutput, speed);
    }

    public Command intake() {
        return runEnd(() -> {
            m_bumperMotor.set(ControlMode.PercentOutput, BumperConstants.bumperSpeedIn);
        }, () -> {
            m_bumperMotor.set(ControlMode.PercentOutput, 0);
        });
    }
    
    public Command release() {
        return run(() -> {
            m_bumperMotor.set(ControlMode.PercentOutput, BumperConstants.bumperSpeedOut);
        });
    }
}
