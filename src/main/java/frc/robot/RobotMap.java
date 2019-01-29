/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int JOYSTICK_DRIVE_VERTICAL = 0;
  public static final int JOYSTICK_DRIVE_HORIZONTAL = 1;
  public static final int JOYSTICK_GAMEPAD = 2;

  public static class Drivetrain {
    // Right: 15, 16, 17 Left: 6, 7, 12
    public static final int[] DRIVETRAIN_MOTOR_PORTS = {15, 16, 17, 6, 7, 12};
    public static final int[] DRIVE_RIGHT_ENCODER = { 2, 3 };
    public static final int[] DRIVE_LEFT_ENCODER = { 0, 1 };
  }

  public static class Lift {
    public static final int LIFT_MOTOR_1_PORT = 17;
    public static final int LIFT_MOTOR_2_PORT = 18;
    public static final int LIFT_MOTOR_3_PORT = 19;
  }

  public static class CargoDeploy {
    public static final int rightMotor = 12;
    public static final int leftMotor = 13;
  }
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
