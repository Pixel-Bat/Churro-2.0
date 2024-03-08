package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.PivotID;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase{

    private final CANSparkMax m_leftPivot = new CANSparkMax(PivotID.leftPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    //private final CANSparkMax m_rightPivot = new CANSparkMax(PivotID.rightPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);

    private final PIDController controller = new PIDController(PivotConstants.PIDConstants.kP, PivotConstants.PIDConstants.kI, PivotConstants.PIDConstants.kD);

    private double desiredPos;
    private double maintainPos;

    private DoubleSupplier currentEncoderPos;
    private DoubleSupplier desiredPivotPos;

    private BooleanSupplier balancingPIDSupplier;
    private boolean isPIDBalancing = true;

   

    public Pivot() {
        //m_rightPivot.setInverted(true);
        m_leftPivot.setInverted(false);
        
        //m_rightPivot.follow(m_leftPivot, true);
        m_encoder.setDistancePerRotation(360/2);
        desiredPos = m_encoder.getAbsolutePosition();

        desiredPivotPos = () -> desiredPos;
        currentEncoderPos = () -> m_encoder.getAbsolutePosition();
        balancingPIDSupplier = () -> isPIDBalancing;
        

       
        Shuffleboard.getTab("Test Tab").add(this);
        Shuffleboard.getTab("Test Tab").add(this.controller);
        //SmartDashboard.putNumber("Pivot", encoderInDegrees());
        //SmartDashboard.putData("Pivot PID", this.controller);
        SmartDashboard.putNumber("SetPos", desiredPos);
        Shuffleboard.getTab("Test Tab").addDouble("Desired Pos", desiredPivotPos);
        Shuffleboard.getTab("Test Tab").addDouble("Current Pos", currentEncoderPos);
        Shuffleboard.getTab("Test Tab").addBoolean("Is Balancing", balancingPIDSupplier);
        // setPivotAngle(-RobotContainer.operatorController.getY());
    }

    // public Command intakePos() {
    //     return runOnce(() -> {
    //         isPIDBalancing = false;
    //         desiredPos = PivotConstants.positions.intakePos;
    //         this.controller.setSetpoint(desiredPos);
    //     });
    // }

    // public Command shootPos() {
    //     return runOnce(() -> {
    //         isPIDBalancing = false;
    //         desiredPos = PivotConstants.positions.shootPos;
    //         this.controller.setSetpoint(desiredPos);
    //     });
    // }

    // public Command groundPos() {
    //     return runOnce(() -> {
    //         isPIDBalancing = false;
    //         desiredPos = PivotConstants.positions.groundPos;
    //         this.controller.setSetpoint(desiredPos);
    //     });
    // }

    public Command customPos(double input) {
        return runEnd(() -> {
            isPIDBalancing = false;
             SmartDashboard.putBoolean("BalancingBool", isPIDBalancing);
            // if (desiredPos > PivotConstants.positions.minPos && desiredPos < PivotConstants.positions.maxPos) {
                desiredPos += input / 5;
                this.controller.setSetpoint(desiredPos);
            //}
        }, () -> {
            isPIDBalancing = true;
        });
    }


    public void periodic() {
        //m_rightPivot.set(this.controller.calculate(encoderToDegrees(m_encoder.getAbsolutePosition())));
        desiredPivotPos = () -> desiredPos;
        currentEncoderPos = () -> m_encoder.getAbsolutePosition();
        balancingPIDSupplier = () -> isPIDBalancing;
        SmartDashboard.putBoolean("BalancingBool", isPIDBalancing);
        m_leftPivot.set(this.controller.calculate(0.5));
    }



    public double encoderInDegrees() {
        return m_encoder.getAbsolutePosition() * 180;
    }

    // public void setPivotAngle(double input) {
    //     //if (encoderInDegrees() > PivotConstants.positions.minPos && encoderInDegrees() < PivotConstants.positions.maxPos) {
    //         m_leftPivot.set(input/5);
        
    //         //m_rightPivot.set(input/10);
    //     //}
    // }
}
