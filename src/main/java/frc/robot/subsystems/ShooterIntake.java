package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.IntakeID;



public class ShooterIntake extends SubsystemBase{
    

    private final CANSparkMax shooterMotor = new CANSparkMax(IntakeID.intakeMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final double speed_in = 0.50;
    private final double speed_out = 1;

    public ShooterIntake() {
        
    }

    public Command intake() {
        return runEnd(() -> {
            shooterMotor.set(speed_in);
        }, () -> {
            shooterMotor.set(0);
        });
    }

    public Command shoot() {
        return runEnd(() -> {
            shooterMotor.set(speed_out);
        }, () -> {
            shooterMotor.set(0);
        });
    }
}
