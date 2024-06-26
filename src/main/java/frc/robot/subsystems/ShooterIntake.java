package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.MotorCANID.IntakeID;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;



public class ShooterIntake extends SubsystemBase{
    

    private final CANSparkMax m_shooterMotor = new CANSparkMax(IntakeID.shooterMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    public static final CommandJoystick operatorController = new CommandJoystick(OperatorConstants.kOperatorControllerPort);

    

    public ShooterIntake() {

        // factory reset spark max and set current limit
        m_shooterMotor.restoreFactoryDefaults();
        m_shooterMotor.setSmartCurrentLimit(RobotConstants.driveCurrentLimit);
        
    }

    public void setShooterSpeed(double input) {
        m_shooterMotor.set(input);
    }

    public Command intake() {
        return runEnd(() -> {
            m_shooterMotor.set(ShooterConstants.shooterSpeedIn);
        }, () -> {
            m_shooterMotor.set(0);
        });
    }

    public Command shoot() {
        return runEnd(() -> {
            m_shooterMotor.set(ShooterConstants.shooterSpeedOut * Math.abs(operatorController.getRawAxis(3) - 1));
        }, () -> {
            m_shooterMotor.set(0);
        });
    }

    public Command amp() {
        return runEnd(() -> {
            m_shooterMotor.set(ShooterConstants.ampSpeedOut);
        }, () -> {
            m_shooterMotor.set(0);
        });
    }





    public Command intakeNoEnd() {
        return run(() -> {
            m_shooterMotor.set(ShooterConstants.shooterSpeedIn);
        });
    }

    public Command shootNoEnd() {
        System.out.println("testest");
        return runOnce(() -> {
            m_shooterMotor.set(ShooterConstants.shooterSpeedOut);
        }       
        );
    }

    public Command stop() {
        return runOnce(() -> {
            m_shooterMotor.set(0);
        });
    }
}
