package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
   
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.IntakeID;
import frc.robot.Constants.ShooterConstants;

public class NoteHolder extends SubsystemBase{
    
    
    private final TalonSRX holdingMotor = new TalonSRX(IntakeID.holdingMotorCANID);
    
    
    public NoteHolder() {
        
    }
    
    public Command intake() {
        return runEnd(() -> {
            holdingMotor.set(ControlMode.PercentOutput, ShooterConstants.holderSpeedIn);
        }, () -> {
            holdingMotor.set(ControlMode.PercentOutput, 0);
        });
    }
    
    public Command shoot() {
        return runEnd(() -> {
            holdingMotor.set(ControlMode.PercentOutput, ShooterConstants.holderSpeedOut);
        }, () -> {
            holdingMotor.set(ControlMode.PercentOutput, 0);
        });
    }
}

