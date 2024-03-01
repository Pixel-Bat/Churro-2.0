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
  }

  public static class RobotConstants {
    public static final double GEARBOX_STAGE_1 = 8.46;
    public static final double WHEEL_DIAMETER_IN = 6;
    public static double ROBOT_WIDTH = 21.875; //width between wheels in inches
  }

  public static class MotorCANID {
    public class DrivetrainID {
      // Can IDs for 2023 Robot, change when we make a new one
      public static final int frontLeftMotorCANID = 4;
      public static final int frontRightMotorCANID = 2;
      public static final int backLeftMotorCANID = 5;
      public static final int backRightMotorCANID = 3;
      
    }
    
    public class IntakeID {
      public static final int intakeMotorCANID = 11;
      public static final int holdingMotorCANID = 12;
    }
    
    public class PivotID {
      public static final int leftPivotMotorCANID = 6;
      public static final int rightPivotMotorCANID = 7;
    }
  }

  public static class PivotConstants {
    public class PIDConstants {
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  
    public class positions {
      public static final int intakePos = 150;
      public static final int shootPos = 125;
      public static final int maxPos = 170;
      public static final int minPos = 60;
    }
  }

}
