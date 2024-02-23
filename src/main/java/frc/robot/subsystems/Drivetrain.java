package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d(5.0, 13.5, new Rotation2d()));
    private Field2d m_field = new Field2d();

    public Drivetrain() {
        // if the code is runing on the robot, define the motors as brushed, but if it is in the sim, then define them as brusless to prevent the error when simulating the robot

        if (Robot.isReal()){
            // Define the motor ports and motor types as brushed
            this.frontLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.frontRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.backLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.backRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        } else {
            // Define the motor ports and motor types as brushless
            this.frontLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.frontRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.backLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
            this.backRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        }
        
        // Link the motors on the right and left
        this.backLeftMotor.follow(frontLeftMotor);
        this.backRightMotor.follow(frontRightMotor);

        // Set the differential drivetrain
        this.m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        // Update simualtion field data
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Runs the robot in arcade drive mode at the given speed and with the rotation
     * @param forward
     * @param rotation
     */
    public void Drive(double throttle, double rotation) {
        m_drive.arcadeDrive(throttle, rotation);
    }

    // // Update robot position for the sim
    public void simulationPeriodic() {
        // Set the simulation differential drivetrain (placeholder values to change later)
        DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 7.29, 7.5, 60.0, Units.inchesToMeters(3), 0.7112, VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
        
        m_driveSim.setInputs(
            this.frontLeftMotor.get() * RobotController.getBatteryVoltage() / 5,
            this.frontRightMotor.get() * RobotController.getBatteryVoltage() / 5);
        m_driveSim.update(0.020);


        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

        SmartDashboard.putData("Field", m_field);
    }

    public void periodic() {
        // Update the robot position
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }
}
