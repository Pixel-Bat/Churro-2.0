package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase{

private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;
  private final CANSparkMax backLeftMotor;
  private final CANSparkMax backRightMotor;

  private final DifferentialDrive drive;

    public Drivetrain() {

        // if the code is runing on the robot, define the motors as brushed, but if it is in the sim, then define them as brusless to prevent the error when simulating the robot

        if (Robot.isReal()){
            // Define the motor ports and motor types as brushed
            this.frontLeftMotor = new CANSparkMax(Constants.MotorCANID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushed);
            this.frontRightMotor = new CANSparkMax(Constants.MotorCANID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushed);
            this.backLeftMotor = new CANSparkMax(Constants.MotorCANID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushed);
            this.backRightMotor = new CANSparkMax(Constants.MotorCANID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushed);
        } else {
            // Define the motor ports and motor types as brushless
            this.frontLeftMotor = new CANSparkMax(Constants.MotorCANID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.frontRightMotor = new CANSparkMax(Constants.MotorCANID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.backLeftMotor = new CANSparkMax(Constants.MotorCANID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.backRightMotor = new CANSparkMax(Constants.MotorCANID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        }
        
        
        // Link the motors on the right and left
        this.backLeftMotor.follow(frontLeftMotor);
        this.backRightMotor.follow(frontRightMotor);

        // Set the differential drivetrain
        this.drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    }

    /**
     * Runs the robot in arcade drive mode at the given speed and with the rotation
     * @param forward
     * @param rotation
     */
    public Command drive(double forward, double rotation) {
        return runEnd(() -> {
            this.drive.arcadeDrive(forward, rotation);
        }, this.drive::stopMotor);
    }
}
