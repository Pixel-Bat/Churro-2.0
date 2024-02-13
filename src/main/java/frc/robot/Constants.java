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

    // Buttons
    public static final int A_BUTTON = 1;  
    public static final int B_BUTTON = 2; 
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6; 
    public static final int BACK = 7;
    public static final int START = 8; 
    public static final int LEFT_STICK = 9;
    public static final int RIGHT_STICK = 10;

    public static final int POV_UP = 0;
    public static final int POV_LEFT = 270;
    public static final int POV_DOWN = 180;
    public static final int POV_RIGHT = 90;

    // Axis
    public static final int LEFT_X = 0;
    public static final int LEFT_Y = 1;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
    public static final int RIGHT_X = 4;
    public static final int RIGHT_Y = 5;
    
     // Trigger Control
    public static final double DIGITAL_TRIGGER_THRESHOLD = 0.85;

    public static final double JOYSTICK_DEADBAND= 0.05;
    public static final double THROTTLE_FACTOR = -1.0;
    public static final double ROTATION_FACTOR = -0.8;
  }

  public static class RobotConstants {
    public static final double GEARBOX_STAGE_1 = 0;
    public static final double WHEEL_DIAMETER_IN = 0;
    public static final double GEARBOX_STAGE_2 = 0;
    public static final double PULLEY_STAGE = 0;
    public static double ROBOT_WIDTH = 0.0; //put actual value here
  }

  public static class MotorCANID {
    public class DrivetrainID {
      // Can IDs for 2023 Robot, change when we make a new one
      public static final int frontLeftMotorCANID = 3;
      public static final int frontRightMotorCANID = 1;
      public static final int backLeftMotorCANID = 4;
      public static final int backRightMotorCANID = 2;
      
    }
    
    public static class IntakeID {
      public static final int intakeMotorCANID = 11;
    }
    
  }
}
