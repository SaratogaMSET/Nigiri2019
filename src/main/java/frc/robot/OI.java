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
import frc.robot.util.filters.JoystickFilter;
import frc.robot.util.filters.JoystickFilter.Mode;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public Joystick driverVertical;
  public Joystick driverHorizontal;
  public Joystick gamepad;
  public Joystick operatorJoystick;

  public Button visionFixButton;

  public Driver driver;
  public Gamepad gamePad;
  public Operator operator;

  public OI() {
    driverHorizontal = new Joystick(RobotMap.JOYSTICK_DRIVE_HORIZONTAL);
    driverVertical = new Joystick(RobotMap.JOYSTICK_DRIVE_VERTICAL);
    gamepad = new Joystick(RobotMap.JOYSTICK_GAMEPAD);
    operatorJoystick = new Joystick(RobotMap.OPERATOR_JOYSTICK);

    driver = new Driver();
    gamePad = new Gamepad();
    operator = new Operator();

    visionFixButton = new JoystickButton(driverHorizontal, 1); // driver rot stick trigger
  }

  public class Driver {
    private JoystickFilter verticalFilter = new JoystickFilter(0.1, 0.0, 1.0, Mode.LINEAR);
    private JoystickFilter horizontalFilter = new JoystickFilter(0.1, 0.0, 1.0, Mode.LINEAR);

    public double getDriverVertical() {
      double y = -driverVertical.getY(); // Y is reversed
      return verticalFilter.filter(y);
    }

    public double getDriverHorizontal() {
      return horizontalFilter.filter(driverHorizontal.getX());
    }
    
    public boolean getDriverButton1() {
      return driverVertical.getRawButton(1) || driverHorizontal.getRawButton(1);
    }

    public boolean getDriverButton2() {
      return driverVertical.getRawButton(2) || driverHorizontal.getRawButton(2);
    }

    public boolean getDriverVerticalB1() {
      return driverVertical.getRawButton(1);
    }
    
    public boolean getDriverButton3() {
      return driverVertical.getRawButton(3) || driverHorizontal.getRawButton(3);
    }

    public boolean getDriverButton4() {
      return driverVertical.getRawButton(4) || driverHorizontal.getRawButton(4);
    }

    public boolean getDriverButton5() {
      return driverVertical.getRawButton(5) || driverHorizontal.getRawButton(5);
    }

    public boolean getDriverButton6() {
      return driverVertical.getRawButton(6) || driverHorizontal.getRawButton(6);
    }

    public boolean getDriverButton7() {
      return driverVertical.getRawButton(7) || driverHorizontal.getRawButton(7);
    }

    public boolean getDriverButton8() {
      return driverVertical.getRawButton(8) || driverHorizontal.getRawButton(8);
    }
  }

  public class Operator{
    public double getY(){
      if(Math.abs(operatorJoystick.getY()) > 0.05){
        return -operatorJoystick.getY();
      }
      return 0.0;
    }
    public boolean getButton7(){
      return operatorJoystick.getRawButton(7);
    }
    public boolean getButton8(){
      return operatorJoystick.getRawButton(8);
    }
  }

  public class Gamepad {
    public boolean getButtonA() {
      return gamepad.getRawButton(1);
    }

    public boolean getButtonAPressed() {
      return gamepad.getRawButtonPressed(1);
    }

    public boolean getButtonAReleased() {
      return gamepad.getRawButtonReleased(1);
    }

    public boolean getButtonB() {
      return gamepad.getRawButton(2);
    }

    public boolean getButtonBReleased() {
      return gamepad.getRawButtonReleased(2);
    }

    public boolean getButtonBPressed() {
      return gamepad.getRawButtonPressed(2);
    }

    public boolean getButtonX() {
      return gamepad.getRawButton(3);
    }

    public boolean getButtonXPressed() {
      return gamepad.getRawButtonPressed(3);
    }

    public boolean getButtonXReleased() {
      return gamepad.getRawButtonReleased(3);
    }

    public boolean getButtonY() {
      return gamepad.getRawButton(4);
    }

    public boolean getButtonYPressed() {
      return gamepad.getRawButtonPressed(4);
    }

    public boolean getButtonYReleased() {
      return gamepad.getRawButtonReleased(4);
    }

    public boolean getLeftButton() {
      return gamepad.getRawButton(5);
    }

    public boolean getLeftButtonPressed() {
      return gamepad.getRawButtonPressed(5);
    }

    public boolean getLeftButtonReleased() {
      return gamepad.getRawButtonReleased(5);
    }

    public boolean getRightButton() {
      return gamepad.getRawButton(6);
    }

    public boolean getRightButtonReleased() {
      return gamepad.getRawButtonReleased(6);
    }

    public boolean getRightButtonPressed() {
      return gamepad.getRawButtonPressed(6);
    }

    public boolean getBackButton() {
      return gamepad.getRawButton(7);
    }

    public boolean getBackButtonPressed() {
      return gamepad.getRawButtonPressed(7);
    }

    public boolean getBackButtonReleased() {
      return gamepad.getRawButtonReleased(7);
    }

    public boolean getStartButton() {
      return gamepad.getRawButton(8);
    }

    public boolean getStartButtonPressed() {
      return gamepad.getRawButtonPressed(8);
    }

    public boolean getStartButtonReleased() {
      return gamepad.getRawButtonReleased(8);
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
