package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.IntakeID;

public class Intake extends SubsystemBase{
    private final TalonSRX intakeMotor;

    private final double speed_in = 0.50;
    private final double speed_out = -0.85;

    public Intake() {
        this.intakeMotor = new TalonSRX(IntakeID.intakeMotorCANID);
    }

    public Command grab() {
        return runEnd(() -> {
            intakeMotor.set(ControlMode.PercentOutput, speed_in);
        }, () -> {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        });
    }

    public Command release() {
        return runEnd(() -> {
            intakeMotor.set(ControlMode.PercentOutput, speed_out);
        }, () -> {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        });
    }
}
