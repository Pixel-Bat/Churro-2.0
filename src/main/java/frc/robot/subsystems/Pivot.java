package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.PivotID;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase{

    private final CANSparkMax leftPivot = new CANSparkMax(PivotID.leftPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightPivot = new CANSparkMax(PivotID.rightPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    private final PIDController controller = new PIDController(PivotConstants.PIDConstants.kP, PivotConstants.PIDConstants.kI, PivotConstants.PIDConstants.kD);

    private double currentPos;

    public Pivot() {
        rightPivot.setInverted(true);
        encoder.setDistancePerRotation(360);
        
    }

    public Command intakePos() {
        return runOnce(() -> {
            currentPos = PivotConstants.positions.intakePos;
            this.controller.setSetpoint(currentPos);
        });
    }

    public Command shootPos() {
        return runOnce(() -> {
            currentPos = PivotConstants.positions.shootPos;
            this.controller.setSetpoint(currentPos);
        });
    }

    public Command customPos(double input) {
        return run(() -> {
            if (currentPos > PivotConstants.positions.minPos && currentPos < PivotConstants.positions.maxPos) {
                currentPos += input;
                this.controller.setSetpoint(currentPos);
            }
        });
    }


    public void Periodic() {
        rightPivot.set(this.controller.calculate(encoder.get()));
        leftPivot.set(this.controller.calculate(encoder.get()));
    }
}
