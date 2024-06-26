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
    
    public void setHolderSpeed(double speed) {
        holdingMotor.set(ControlMode.PercentOutput, speed);
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

    public Command amp() {
        return runEnd(() -> {
            holdingMotor.set(ControlMode.PercentOutput, ShooterConstants.holderSpeedAmp);
        }, () -> {
            holdingMotor.set(ControlMode.PercentOutput, 0);
        });
    }


    public Command intakeNoEnd() {
        return run(() -> {
            holdingMotor.set(ControlMode.PercentOutput, ShooterConstants.holderSpeedIn);
        });
    }

    public Command shootNoEnd() {
        return run(() -> {
            holdingMotor.set(ControlMode.PercentOutput, ShooterConstants.holderSpeedOut);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            holdingMotor.set(ControlMode.PercentOutput, 0);
        });
    }
}

