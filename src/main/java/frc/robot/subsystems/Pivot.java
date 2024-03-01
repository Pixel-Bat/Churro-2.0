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

    private final CANSparkMax m_leftPivot = new CANSparkMax(PivotID.leftPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_rightPivot = new CANSparkMax(PivotID.rightPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);

    private final PIDController controller = new PIDController(PivotConstants.PIDConstants.kP, PivotConstants.PIDConstants.kI, PivotConstants.PIDConstants.kD);

    private double currentPos;

    public Pivot() {
        m_rightPivot.setInverted(true);
        m_encoder.setDistancePerRotation(360);
        currentPos = m_encoder.get();
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
        m_rightPivot.set(this.controller.calculate(m_encoder.get()));
        m_leftPivot.set(this.controller.calculate(m_encoder.get()));
    }
}
