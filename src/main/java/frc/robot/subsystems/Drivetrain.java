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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

    // Simulated objects for the sim odometry
    private final Encoder m_leftEncoder = new Encoder(1, 2);
    private final Encoder m_rightEncoder = new Encoder(3, 4);
    private final AnalogGyro m_gyro = new AnalogGyro(1);
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d(5.0, 13.5, new Rotation2d()));
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), 7.29, 7.5, 60.0, Units.inchesToMeters(3), 0.7112, VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
    private Field2d m_field = new Field2d();

    public Drivetrain() {

<<<<<<< Updated upstream
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
        
=======
        // if the code is runing on the robot, define the motors as brushed, but if it
        // is in the sim, then define them as brusless to prevent the error when
        // simulating the robot

        if (Robot.isReal()) {
            // Define the motor ports and motor types as brushed
            this.frontLeftMotor = new CANSparkMax(Constants.MotorCANID.frontLeftMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushed);
            this.frontRightMotor = new CANSparkMax(Constants.MotorCANID.frontRightMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushed);
            this.backLeftMotor = new CANSparkMax(Constants.MotorCANID.backLeftMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushed);
            this.backRightMotor = new CANSparkMax(Constants.MotorCANID.backRightMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushed);
        } else {
            // Define the motor ports and motor types as brushless
            this.frontLeftMotor = new CANSparkMax(Constants.MotorCANID.frontLeftMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushless);
            this.frontRightMotor = new CANSparkMax(Constants.MotorCANID.frontRightMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushless);
            this.backLeftMotor = new CANSparkMax(Constants.MotorCANID.backLeftMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushless);
            this.backRightMotor = new CANSparkMax(Constants.MotorCANID.backRightMotorCANID,
                    CANSparkLowLevel.MotorType.kBrushless);
        }
>>>>>>> Stashed changes
        
        // Link the motors on the right and left
        this.backLeftMotor.follow(frontLeftMotor);
        this.backRightMotor.follow(frontRightMotor);

        // Set the differential drivetrain
        this.drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

        // Update simualtion field data
        SmartDashboard.putData("Field", m_field);
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

    // // Update robot position for the sim
    public void simulationPeriodic() {
        
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putData("Field", m_field);
    }
}
