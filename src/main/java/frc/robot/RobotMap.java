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
  public static final int OPERATOR_JOYSTICK = 2; 
  public static final int JOYSTICK_GAMEPAD = 3;
 
  public static class Drivetrain {
    // Right: 16, 17, 20 Left: 6, 7, 11
    public static final int[] DRIVETRAIN_MOTOR_PORTS = {16, 17, 20, 6, 7, 11};
    public static final int[] DRIVE_RIGHT_ENCODER = { 2, 3 };
    public static final int[] DRIVE_LEFT_ENCODER = { 0, 1 };
  }

  public static class Lift {
    public static final int LIFT_MOTOR_1_PORT = 21;
    public static final int LIFT_MOTOR_2_PORT = 18;
    public static final int LIFT_MOTOR_3_PORT = 19;
    public static final int BOTTOM_HAL_EFFECT = 8;
    public static final int TOP_HAL_EFFECT = 7;
  }

  public static class CargoDeploy {
    public static final int DEPLOY_INTAKE_MOTOR = 12;
    public static final int IR_SENSOR = 10;
    public static final int INTAKE_SOL = 2;
  }
  public static class Jacks {
    public static final int JACK_MOTOR = 13;
    public static final int JACK_DRIVE_MOTOR = 14;
    public static final int DOWN_HAL = 4;
    public static final int UP_HAL = 6;
  }

  public static class CargoIntake {
    public static final int RIGHT_INTAKE = 9;
    public static final int LEFT_INTAKE = 8;
    public static final int BACK_INTAKE = 22;
    public static final int[] INTAKE_SOL = { 0, 1 };
    public static final int INTAKE_DOWN_HAL = 5;
    public static final int INTAKE_UP_HAL = 9;
  }

  public static class Hatch {
    public static final int[] HATCH_PISTON = { 4, 5 };
    public static final int[] HATCH_DEPLOY_PISTON_1 = { 2, 3 };
    public static final int[] HATCH_DEPLOY_PISTON_2 = { 6, 7 };
    public static final int HATCH_SWITCH = 1;
  }

  public static class Ultrasonic {
    public static final int ULTRASONIC_LEFT = 0;
    public static final int ULTRASONIC_RIGHT = 1;
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
