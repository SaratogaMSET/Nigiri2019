/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private Joystick driverVertical;
  private Joystick driverHorizontal;
  private Joystick gamepad;

  public Button visionFixButton;

  public Driver driver;
  public Gamepad gamePad;

  public OI() {
    driverHorizontal = new Joystick(RobotMap.JOYSTICK_DRIVE_HORIZONTAL);
    driverVertical = new Joystick(RobotMap.JOYSTICK_DRIVE_VERTICAL);
    gamepad = new Joystick(RobotMap.JOYSTICK_GAMEPAD);

    driver = new Driver();
    gamePad = new Gamepad();

    visionFixButton = new JoystickButton(driverHorizontal, 1);
  }

  public class Driver {
    public double getDriverVertical() {
      return -driverVertical.getY();
    }
    

    public boolean getDriverButton1() {
      return driverVertical.getRawButton(1) || driverHorizontal.getRawButton(1);
    }

    public boolean getDriverButton2() {
      return driverVertical.getRawButton(2);
    }

    public double getDriverHorizontal() {
      return driverHorizontal.getX();
    }

    /**
     * Interprets joystick data and performs filtering/squaring/smoothing operations to feed power to the drivetrain.
     * @return Returns a double array of length two:[0] is the normalized (-1.0 to 1.0) power of the left, [1] is of the right
     */
    public double[] getArcadePower() {
      double fwd = getDriverVertical();
      double rot = getDriverHorizontal();

      double leftRaw = fwd + rot;
      double rightRaw = fwd - rot;

      double normalizer = Math.max(1, Math.max(Math.abs(leftRaw), Math.abs(rightRaw)));
      
      double leftNormalized  = leftRaw / normalizer;
      double rightNormalized = rightRaw / normalizer;

      // leftNormalized = Math.pow(leftNormalized, 3.0);
      // rightNormalized = Math.pow(rightNormalized, 3.0);

      return new double[] {leftNormalized, rightNormalized};
    }
  }

  public class Gamepad {
    public boolean getButtonA() {
      return gamepad.getRawButton(1);
    }

    public boolean getButtonB() {
      return gamepad.getRawButton(2);
    }

    public boolean getButtonX() {
      return gamepad.getRawButton(3);
    }

    public boolean getButtonY() {
      return gamepad.getRawButton(4);
    }

    public boolean getLeftButton() {
      return gamepad.getRawButton(5);
    }

    public boolean getRightButton() {
      return gamepad.getRawButton(6);
    }

    public boolean getBackButton() {
      return gamepad.getRawButton(7);
    }

    public boolean getStartButton() {
      return gamepad.getRawButton(8);
    }

    public boolean getLeftJoystickButton() {
      return gamepad.getRawButton(9);
    }

    public boolean getRightJoystickButton() {
      return gamepad.getRawButton(10);
    }

    public double getRightJoystickX() {
      return gamepad.getRawAxis(4);
    }

    public double getRightJoystickY() {
      return -gamepad.getRawAxis(5);
    }

    public double getLeftJoystickX() {
      return gamepad.getRawAxis(0);
    }

    public double getLeftJoystickY() {
      return -gamepad.getRawAxis(1);
    }

    public double getRightTrigger() {
      return gamepad.getRawAxis(3);
    }

    public double getLeftTrigger() {
      return gamepad.getRawAxis(2);
    }

    public int getPOV() {
      return gamepad.getPOV();
    }
  }
  
}
