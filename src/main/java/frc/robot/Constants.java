// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    // Ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final String operatorShuffleboardTab = "Driver Station";
  }

  public static class RobotConstants {
    public static final double GEARBOX_STAGE_1 = 1/8.46;
    public static final double WHEEL_DIAMETER_IN = 6;
    public static final double ROBOT_WIDTH = 21.875; //width between wheels in inches

    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ACCELERATION = 0.5;

    
    public static final int driveCurrentLimit = 60;
  }

  public static class MotorCANID {
    public class DrivetrainID {
      // Can IDs for 2024 Robot, change when we make a new one
      public static final int frontLeftMotorCANID = 4;
      public static final int frontRightMotorCANID = 2;
      public static final int backLeftMotorCANID = 5;
      public static final int backRightMotorCANID = 3;
      
    }
    
    public class IntakeID {
      public static final int shooterMotorCANID = 8;
      public static final int holdingMotorCANID = 10;
      public static final int bumperIntakeMotorCANID = 12;
    }


    
    
    public class PivotID {
      public static final int leftPivotMotorCANID = 6;
      public static final int rightPivotMotorCANID = 7;
    }
  }

  public static class ShooterConstants {

    public static final int shooterCurrentLimit = 60;

    public static final double shooterSpeedIn = -0.6;
    public static final double shooterSpeedOut = 1;
    public static final double ampSpeedOut = 0.1;

    public static final double holderSpeedIn = -0.75;
    public static final double holderSpeedOut = 0.75;
    public static final double holderSpeedAmp = 0.25;
  }

  public static class BumperConstants {


    public static final double bumperSpeedIn = 1;
    public static final double bumperSpeedOut = -1;
    //public static final double ampSpeedOut = 0.1;

    //public static final double holderSpeedIn = -0.5;
    //public static final double holderSpeedOut = 0.5;
  }

  public static class PivotConstants {

    public static final int pivotCurrentLimit = 40;

    public class PIDConstants {
      public static final double kP = 0.007;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  
    public class positions {
      public static final int intakePos = 150;
      public static final int shootPos = 125;
      public static final int maxPos = 75;
      public static final int minPos = 15;

      public static final double sourceHeightMin = 52;
      public static final double sourceHeightMax = 57.2;

      public static final double speakerHeightMin = 70.5;
      public static final double speakerHeightMax = 73;

      public static final double speakerHeight = 151;
      public static final double sourceHeight = 128;
      public static final double ampHeight = 137;
      public static final double ampHeight2 = 112;
      public static final double intakeHeight = 16;

      public static final double flatHeight = 91;
    }
  }

}
