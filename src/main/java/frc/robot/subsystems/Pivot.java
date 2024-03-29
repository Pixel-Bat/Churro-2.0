package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID.PivotID;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.RobotConstants;

public class Pivot extends SubsystemBase{

    private final CANSparkMax m_leftPivot = new CANSparkMax(PivotID.leftPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    //private final CANSparkMax m_rightPivot = new CANSparkMax(PivotID.rightPivotMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);

    public final PIDController leftPivotController = new PIDController(PivotConstants.PIDConstants.kP, PivotConstants.PIDConstants.kI, PivotConstants.PIDConstants.kD);

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

        // factory reset spark max
        m_leftPivot.restoreFactoryDefaults();
        //m_rightPivot.restoreFactoryDefaults();

        // set current limit for spark max
        m_leftPivot.setSmartCurrentLimit(RobotConstants.driveCurrentLimit);
        //m_rightPivot.setSmartCurrentLimit(RobotConstants.driveCurrentLimit);


        // invert right pivot motor
        //m_rightPivot.setInverted(true);
        m_leftPivot.setInverted(false);
        
        //m_rightPivot.follow(m_leftPivot, true);
        m_encoder.setDistancePerRotation(360/2);
        intakeCheckBoolSupplier = () -> intakeBool;
        speakerCheckBoolSupplier = () -> speakerBool;

        intakeUpBoolSupplier = () -> intakeUpBool;
        speakerUpBoolSupplier = () -> speakerUpBool;
        intakeDownBoolSupplier = () -> intakeDownBool;
        speakerDownBoolSupplier = () -> speakerDownBool;

        encoderMarginSupplier = () -> encoderMargins;

        outputSupplier = () -> output;
        encoderAngle = () -> encoderInDegrees();

        leftPivotController.setTolerance(encoderMargins, 0.04);

        setPivotAngle(PivotConstants.positions.intakeHeight);

        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Source Height", intakeCheckBoolSupplier);
        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Speaker Height", speakerCheckBoolSupplier);

        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Source Up", intakeUpBoolSupplier);
        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Speaker Up", speakerUpBoolSupplier);
        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Source Down", intakeDownBoolSupplier);
        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addBoolean("Speaker Down", speakerDownBoolSupplier);

        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addNumber("Pivot Output", outputSupplier);

        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addNumber("Angle", encoderAngle);
        
        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).addNumber("EncoderMargin", encoderMarginSupplier);

        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).add("Pivot", m_encoder);
        Shuffleboard.getTab(OperatorConstants.operatorShuffleboardTab).add("PID", this.leftPivotController);
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

        // SmartDashboard.putNumber("Pivot", encoderInDegrees());
        // SmartDashboard.putData(this.controller);
        encoderAngle = () -> encoderInDegrees();
        encoderMarginSupplier = () -> encoderMargins;
        runPID();
        outputSupplier = () -> output;
    }

    public Command autoArmHeight() {
        return run(() -> {
            setPivotAngle((PivotConstants.positions.speakerHeightMax + PivotConstants.positions.speakerHeightMin) / 2);
        });
    }

    public double encoderInDegrees() {
        return m_encoder.getAbsolutePosition() * 180;
    }

    public void setPivotAngle(double input) {
       // if (encoderInDegrees() > PivotConstants.positions.minPos && encoderInDegrees() < PivotConstants.positions.maxPos) {
            // Should be (encoderValue, then setpoint)
            leftPivotController.setSetpoint(input);
            
        //}
    }

    public void runPID() {
        output = leftPivotController.calculate(encoderInDegrees());
        if (output > 0.4)
        output = 0.4;

        if (output < -0.4)
            output = -0.4;

        m_leftPivot.set(-output);
    }


    public Command speakerPos() {
        return run(() -> {
            setPivotAngle(PivotConstants.positions.speakerHeight);
        });
    }

    public Command ampPos() {
        return run(() -> {
            setPivotAngle(PivotConstants.positions.ampHeight);
        });
    }

    public Command sourcePos() {
        return run(() -> {
            setPivotAngle(PivotConstants.positions.sourceHeight);
        });
    }

    public Command intakePos() {
        return run(() -> {
            setPivotAngle(PivotConstants.positions.intakeHeight);
        });
    }

    public Command flatPos() {
        return run(() -> {
            setPivotAngle(PivotConstants.positions.flatHeight);
        });
    }

    public Command ampPos2() {
        return run(() -> {
            setPivotAngle(PivotConstants.positions.ampHeight2);
        });
    }
}
