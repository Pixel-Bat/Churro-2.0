package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCANID;
import frc.robot.Constants.RobotConstants;

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
    private final CANSparkMax m_frontLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontLeftMotorCANID, 
    CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_frontRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontRightMotorCANID, 
    CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax m_backLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.backLeftMotorCANID, 
    CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_backRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.backRightMotorCANID, 
    CANSparkLowLevel.MotorType.kBrushless);

    
      
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

    public Drivetrain() {
        // if the code is runing on the robot, define the motors as brushed, but if it is in the sim, then define them as brusless to prevent the error when simulating the robot


        // Define the motor ports and motor types as brushed
        this.frontLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.frontRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.frontRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.backLeftMotor = new CANSparkMax(MotorCANID.DrivetrainID.backLeftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        this.backRightMotor = new CANSparkMax(MotorCANID.DrivetrainID.backRightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
        

        m_frontLeftMotor.setInverted(true);
        m_backLeftMotor.setInverted(true);

        // Link the motors on the right and left
        this.backLeftMotor.follow(frontLeftMotor);
        this.backRightMotor.follow(frontRightMotor);

        // Set the differential drivetrain
        this.m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        m_backLeftMotor.follow(m_frontLeftMotor);
        m_backRightMotor.follow(m_frontRightMotor);
        

        //remove this when adding gyro and auto stuff
        
        m_isRedAlliance = isRed( DriverStation.getRawAllianceStation());

        configureAutoBuilder();
    }

   
    public void Periodic() {
        m_odometry.update(null, getRightEncoderMeters(), getLeftEncoderMeters());
        m_field.setRobotPose(getPose());
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

    

    
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftOutput =
            m_leftPIDController.calculate(getLeftEncoderMetersPerSecond(), speeds.leftMetersPerSecond);
        final double rightOutput =
            m_rightPIDController.calculate(getRightEncoderMetersPerSecond(), speeds.rightMetersPerSecond);
        m_frontLeftMotor.setVoltage(leftOutput);
        m_frontRightMotor.setVoltage(rightOutput);
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
        double wheelRPS = motorRPS * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
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
        double wheelRotations = motorRotations * RobotConstants.GEARBOX_STAGE_1 * RobotConstants.GEARBOX_STAGE_2 * RobotConstants.PULLEY_STAGE;
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