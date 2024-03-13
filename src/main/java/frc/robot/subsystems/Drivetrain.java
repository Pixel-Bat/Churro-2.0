package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID;
import frc.robot.Constants.RobotConstants;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
public class Drivetrain extends SubsystemBase{




  //private final DifferentialDrive m_drive;

    private final CANSparkMax m_frontLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_frontRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax m_backLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_backRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
    
      
    private final RelativeEncoder m_left = m_frontLeftMotor.getEncoder();
    private final RelativeEncoder m_right = m_frontRightMotor.getEncoder();

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(),
        getLeftEncoderMeters(),
        getRightEncoderMeters()
    );

    private final DifferentialDrive m_drive = new DifferentialDrive(
        m_frontLeftMotor::set, 
        m_frontRightMotor::set
    );

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(
        Units.inchesToMeters(RobotConstants.ROBOT_WIDTH)
    );

    private Field2d m_field = new Field2d();


    private final Boolean m_isRedAlliance;

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

    //private final Shuffleboard shuffleboard = new shuffleboard();

    public Drivetrain() {
        
        
        Shuffleboard.getTab("Test Tab").add(m_gyro);
        Shuffleboard.getTab("Test Tab").add(m_field);
        Shuffleboard.getTab("Test Tab").add(m_drive);
        //Shuffleboard.getTab("Test Tab").addNumber("test", m_left.getPosition());
       
        m_frontLeftMotor.setInverted(true);
        m_backLeftMotor.setInverted(true);

        m_frontRightMotor.setInverted(false);
        m_backRightMotor.setInverted(false);

        
        m_backLeftMotor.follow(m_frontLeftMotor);
        m_backRightMotor.follow(m_frontRightMotor);
        
        
        
        m_isRedAlliance = isRed( DriverStation.getRawAllianceStation());

        configureAutoBuilder();
        
    }

   
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(), getRightEncoderMeters(), getLeftEncoderMeters());
        m_field.setRobotPose(getPose());
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putData("Gyro Heading", m_gyro);
        SmartDashboard.putNumber("Left Encoder", getLeftEncoderMeters());
        SmartDashboard.putNumber("Right Encoder", getRightEncoderMeters());

        m_drive.feed();
        m_drive.feedWatchdog();

        
        // Shuffleboard.getTab("Test Tab").add(m_leftPIDController);
        // Shuffleboard.getTab("Test Tab").add(m_rightPIDController);
        

        

    }

    /**
     * Runs the robot in arcade drive mode at the given speed and with the rotation
     * @param forward
     * @param rotation
     */
    public void Drive(double throttle, double rotation) {
        m_drive.arcadeDrive(throttle, rotation);
    }

    public void arcadeDrive(double throttle, double rotation) {
        m_drive.arcadeDrive(throttle, rotation);
    }


    
    public void driveWithChassisSpeeds(ChassisSpeeds speeds){
        double x = speeds.vxMetersPerSecond;
        double y = speeds.vyMetersPerSecond;
        double rot = speeds.omegaRadiansPerSecond;
        
        var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(x, y, rot));
        setSpeeds(wheelSpeeds);
    }

    

    // This is only called during auto, if you comment it out along with all of the other places it is referenced, it only affects the auto.
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftOutput =
            m_leftPIDController.calculate(getLeftEncoderMetersPerSecond(), speeds.leftMetersPerSecond);
        final double rightOutput =
            m_rightPIDController.calculate(getRightEncoderMetersPerSecond(), speeds.rightMetersPerSecond);
        m_frontLeftMotor.setVoltage(leftOutput);
        m_backLeftMotor.setVoltage(leftOutput);

        m_frontRightMotor.setVoltage(rightOutput);
        m_backRightMotor.setVoltage(rightOutput);

        //Parameters:
            // xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
            // zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.

        /*  I saw others with the same issue saying that the team wasn't using the built in arcadeDrive functions and was using
            setVoltage instead of the built in functions for driving. They said to either use the built in driving functions,
            or to use the DifferentialDrive's built in feed() function which already exists in the arcadeDrive() function.

            My thinking was that we could take the average of the leftOutput + rightOutput as forward, and average of leftOuput
            minus rightOutput for rotation.
        */
        m_drive.arcadeDrive((leftOutput + rightOutput) / 2, (leftOutput - rightOutput) / 2);

        SmartDashboard.putNumber("test", rightOutput);
        SmartDashboard.putNumber("test2", leftOutput);
      }

      
    
    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getRightEncoderMeters(), getLeftEncoderMeters(), pose);
    }

    private ChassisSpeeds getWheelSpeeds(){
        var wheelSpeeds = new DifferentialDriveWheelSpeeds(
        getLeftEncoderMetersPerSecond(), 
        getRightEncoderMetersPerSecond()
        );
        return m_kinematics.toChassisSpeeds(wheelSpeeds);
    }

    private double getLeftEncoderMetersPerSecond(){
       return rpmToMeterPerSecond(m_left.getVelocity());
    }
    
    private double getRightEncoderMetersPerSecond(){
        return rpmToMeterPerSecond(m_right.getVelocity());
    }
    
    private double rpmToMeterPerSecond(double motorRPM){
        double motorRPS = motorRPM/60;
        double wheelRPS = motorRPS * RobotConstants.GEARBOX_STAGE_1;
        double distancePerRev = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
        return wheelRPS * distancePerRev;
    }

    public double getLeftEncoderMeters(){
        return revolutionsToMeters(m_left.getPosition());
    }
    public double getRightEncoderMeters(){
        return revolutionsToMeters(m_right.getPosition());
    }

    private double revolutionsToMeters(double motorRotations){
        double wheelRotations = motorRotations * RobotConstants.GEARBOX_STAGE_1;
        double distancePerRevolution = Units.inchesToMeters(RobotConstants.WHEEL_DIAMETER_IN) * Math.PI;
        return wheelRotations * distancePerRevolution;
    }

    

    
    /* Creates a pathplanner autobuilder for autonomous pathing */
    
    public void configureAutoBuilder() {
        AutoBuilder.configureRamsete(
        this::getPose, 
        this::resetOdometry, 
        this::getWheelSpeeds,
        this::driveWithChassisSpeeds, 
        new ReplanningConfig(),
        this::flipPath, 
        this);
    }

    private boolean flipPath(){
        if (m_isRedAlliance == true){
          return true;
        }
        return false;
    }

    private boolean isRed(AllianceStationID id){
      if(id == AllianceStationID.Red1) return true;
      if(id == AllianceStationID.Red2) return true;
      if(id == AllianceStationID.Red3) return true;
      return false;
    }
}