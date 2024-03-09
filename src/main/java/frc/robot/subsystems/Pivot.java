package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.PivotID;
import frc.robot.Constants.PivotConstants;

import edu.wpi.first.   wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Pivot extends SubsystemBase{

    private final CANSparkMax m_leftPivot = new CANSparkMax(PivotID.leftPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    //private final CANSparkMax m_rightPivot = new CANSparkMax(PivotID.rightPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);

    private final PIDController controller = new PIDController(PivotConstants.PIDConstants.kP, PivotConstants.PIDConstants.kI, PivotConstants.PIDConstants.kD);

    private double desiredPos;

    private double maintainPos;

    private BooleanSupplier intakeCheckBoolSupplier;
    private BooleanSupplier speakerCheckBoolSupplier;

    private BooleanSupplier intakeUpBoolSupplier;
    private BooleanSupplier speakerUpBoolSupplier;
    private BooleanSupplier intakeDownBoolSupplier;
    private BooleanSupplier speakerDownBoolSupplier;

    private DoubleSupplier outputSupplier;
    private DoubleSupplier encoderAngle;

    private boolean intakeBool = false;
    private boolean speakerBool = false;

    private boolean intakeUpBool = false;
    private boolean speakerUpBool = false;
    private boolean intakeDownBool = false;
    private boolean speakerDownBool = false;

    private DoubleSupplier encoderMarginSupplier;
    private double encoderMargins = 0;

    private double output = 0;

   

    public Pivot() {
        //m_rightPivot.setInverted(true);
        m_leftPivot.setInverted(false);
        
        //m_rightPivot.follow(m_leftPivot, true);
        m_encoder.setDistancePerRotation(360/2);
        desiredPos = m_encoder.getAbsolutePosition();
        intakeCheckBoolSupplier = () -> intakeBool;
        speakerCheckBoolSupplier = () -> speakerBool;

        intakeUpBoolSupplier = () -> intakeUpBool;
        speakerUpBoolSupplier = () -> speakerUpBool;
        intakeDownBoolSupplier = () -> intakeDownBool;
        speakerDownBoolSupplier = () -> speakerDownBool;

        encoderMarginSupplier = () -> encoderMargins;

        outputSupplier = () -> output;
        encoderAngle = () -> encoderInDegrees();

        Shuffleboard.getTab("Test Tab").addBoolean("Source Height", intakeCheckBoolSupplier);
        Shuffleboard.getTab("Test Tab").addBoolean("Speaker Height", speakerCheckBoolSupplier);

        Shuffleboard.getTab("Test Tab").addBoolean("Source Up", intakeUpBoolSupplier);
        Shuffleboard.getTab("Test Tab").addBoolean("Speaker Up", speakerUpBoolSupplier);
        Shuffleboard.getTab("Test Tab").addBoolean("Source Down", intakeDownBoolSupplier);
        Shuffleboard.getTab("Test Tab").addBoolean("Speaker Down", speakerDownBoolSupplier);

        Shuffleboard.getTab("Test Tab").addNumber("Pivot Output", outputSupplier);

        Shuffleboard.getTab("Test Tab").addNumber("Angle", encoderAngle);
        
        Shuffleboard.getTab("Test Tab").addNumber("EncoderMargin", encoderMarginSupplier);

        Shuffleboard.getTab("Test Tab").add("Pivot", m_encoder);
        Shuffleboard.getTab("Test Tab").add("PID", this.controller);
    }

    public void periodic() {
        if (encoderInDegrees() >= PivotConstants.positions.sourceHeightMin + encoderMargins && encoderInDegrees() <= PivotConstants.positions.sourceHeightMax) {
            intakeBool = true;
        } else {
            intakeBool = false;
        }

        if (encoderInDegrees() >= PivotConstants.positions.speakerHeightMin + encoderMargins && encoderInDegrees() <= PivotConstants.positions.speakerHeightMax) {
            speakerBool = true;
        } else {
            speakerBool = false;
        }

        if (encoderInDegrees() > PivotConstants.positions.speakerHeightMax + encoderMargins && !speakerBool && !intakeBool) {
            speakerDownBool = true;
        } else {
            speakerDownBool = false;
        }
        if (encoderInDegrees() > PivotConstants.positions.sourceHeightMax + encoderMargins && !speakerBool && !intakeBool) {
            intakeDownBool = true;
        } else {
            intakeDownBool = false;
        }
        if (encoderInDegrees() < PivotConstants.positions.speakerHeightMin + encoderMargins && !speakerBool && !intakeBool) {
            speakerUpBool = true;
        } else {
            speakerUpBool = false;
        }
        if (encoderInDegrees() < PivotConstants.positions.sourceHeightMin + encoderMargins && !speakerBool && !intakeBool) {
            intakeUpBool = true;
        } else {
            intakeUpBool = false;
        }



        SmartDashboard.putNumber("Pivot", encoderInDegrees());
        SmartDashboard.putData(this.controller);
        encoderAngle = () -> encoderInDegrees();
        encoderMarginSupplier = () -> encoderMargins;
        setPivotAngle(-RobotContainer.operatorController.getY());
    }



    public double encoderInDegrees() {
        return m_encoder.getAbsolutePosition() * 180;
    }

    public void setPivotAngle(double input) {
        // //if (encoderInDegrees() > PivotConstants.positions.minPos && encoderInDegrees() < PivotConstants.positions.maxPos) {
            m_leftPivot.set(input/5);
        
        //     //m_rightPivot.set(input/10);
        // //}

        // if (encoderInDegrees() > PivotConstants.positions.minPos && encoderInDegrees() < PivotConstants.positions.maxPos) {
        //     double output = controller.calculate(180);
        //     if (output > 0.2 || output < -0.2) {
        //         output = 0.2;
        //     }

        //     m_leftPivot.set(output / 2);
        // }
    }
}
