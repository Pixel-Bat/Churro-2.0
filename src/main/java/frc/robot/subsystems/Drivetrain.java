package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase{

private final CANSparkMax frontLeftMotor;
  private final CANSparkMax frontRightMotor;
  private final CANSparkMax backLeftMotor;
  private final CANSparkMax backRightMotor;

  private final DifferentialDrive m_drive;

    // Simulated objects for the sim odometry
    private final Encoder m_leftEncoder = new Encoder(0, 1);
    private final Encoder m_rightEncoder = new Encoder(2, 3);
    private final AnalogGyro m_gyro = new AnalogGyro(1);

    public Drivetrain() {
        // if the code is runing on the robot, define the motors as brushed, but if it is in the sim, then define them as brusless to prevent the error when simulating the robot


        // Define the motor ports and motor types as brushed
        this.frontLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.frontRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.backLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.backRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        
        // Link the motors on the right and left
        this.backLeftMotor.follow(frontLeftMotor);
        this.backRightMotor.follow(frontRightMotor);

        // Set the differential drivetrain
        this.m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
    }

    /**
     * Runs the robot in arcade drive mode at the given speed and with the rotation
     * @param forward
     * @param rotation
     */
    public void Drive(double throttle, double rotation) {
        m_drive.arcadeDrive(throttle, rotation);
    }
}