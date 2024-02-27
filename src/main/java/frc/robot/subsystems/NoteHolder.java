package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
   
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.IntakeID;

public class NoteHolder extends SubsystemBase{
    
    
    private final TalonSRX holdingMotor;
    
    private final double speed_in = 0.30;
    private final double speed_out = -0.30;
    
    public NoteHolder() {
        this.holdingMotor = new TalonSRX(IntakeID.holdingMotorCANID);
    }
    
    public Command intake() {
        return runEnd(() -> {
            holdingMotor.set(ControlMode.PercentOutput, speed_in);
        }, () -> {
            holdingMotor.set(ControlMode.PercentOutput, 0);
        });
    }
    
    public Command shoot() {
        return runEnd(() -> {
            holdingMotor.set(ControlMode.PercentOutput, speed_out);
        }, () -> {
            holdingMotor.set(ControlMode.PercentOutput, 0);
        });
    }
}

