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

  public static class POV {
    public static final int LEFT = 270;
    public static final int RIGHT = 90;
    public static final int UP = 0;
    public static final int DOWN = 180;
  }

  public Joystick driverVertical;
  public Joystick driverHorizontal;
  public Joystick gamepad;
  public Joystick operatorJoystick;
  public Joystick buttonBoardRight;
  public Joystick buttonBoardLeft;

  public Button visionFixButton;
  public Button gyroHoldButton;

  public Driver driver;
  public Gamepad gamePad;

  public OI() {
    driverHorizontal = new Joystick(RobotMap.JOYSTICK_DRIVE_HORIZONTAL);
    driverVertical = new Joystick(RobotMap.JOYSTICK_DRIVE_VERTICAL);
    operatorJoystick = new Joystick(RobotMap.OPERATOR_JOYSTICK);

    gamepad = new Joystick(RobotMap.JOYSTICK_GAMEPAD);
  
    driver = new Driver();
    gamePad = new Gamepad();

    visionFixButton = new JoystickButton(driverHorizontal, 1); // driver rot stick trigger
    gyroHoldButton = new JoystickButton(driverVertical, 1); // driver vert stick trigger

  }

  public class Driver {
    private JoystickFilter verticalFilter = new JoystickFilter(0.0, 0.0, 1.0, Mode.LINEAR);
    private JoystickFilter horizontalFilter = new JoystickFilter(0.0, 0.0, 1.0, Mode.SQUARED);

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

    public boolean getDriverButton6Pressed() {
      return driverVertical.getRawButtonPressed(6) || driverHorizontal.getRawButtonPressed(6);
    }

    public boolean getDriverButton6Released() {
      return driverVertical.getRawButtonReleased(6) || driverHorizontal.getRawButtonReleased(6);
    }

    public boolean getDriverButton7() {
      return driverVertical.getRawButton(7) || driverHorizontal.getRawButton(7);
    }

    public boolean getDriverButton7Pressed() {
      return driverVertical.getRawButtonPressed(7) || driverHorizontal.getRawButtonPressed(7);
    }

    public boolean getDriverButton7Released() {
      return driverVertical.getRawButtonReleased(7) || driverHorizontal.getRawButtonReleased(7);
    }

    public boolean getDriverButton8() {
      return driverVertical.getRawButton(8) || driverHorizontal.getRawButton(8);
    }

    public boolean getDriverButton8Pressed() {
      return driverVertical.getRawButtonPressed(8) || driverHorizontal.getRawButtonPressed(8);
    }

    public boolean getDriverButton8Released() {
      return driverVertical.getRawButtonReleased(8) || driverHorizontal.getRawButtonReleased(8);
    }

    public boolean getDriverButton9() {
      return driverVertical.getRawButton(9) || driverHorizontal.getRawButton(9);
    }

    public boolean getDriverButton9Pressed() {
      return driverVertical.getRawButtonPressed(9) || driverHorizontal.getRawButtonPressed(9);
    }

    public boolean getDriverButton9Released() {
      return driverVertical.getRawButtonReleased(9) || driverHorizontal.getRawButtonReleased(9);
    }

    public boolean getDriverButton10() {
      return driverVertical.getRawButton(10) || driverHorizontal.getRawButton(10);
    }

    public boolean getDriverButton10Pressed() {
      return driverVertical.getRawButtonPressed(10) || driverHorizontal.getRawButtonPressed(10);
    }

    public boolean getDriverButton10Released() {
      return driverVertical.getRawButtonReleased(10) || driverHorizontal.getRawButtonReleased(10);
    }

    public boolean getDriverButton11() {
      return driverVertical.getRawButton(11) || driverHorizontal.getRawButton(11);
    }

    public boolean getDriverButton11Pressed() {
      return driverVertical.getRawButtonPressed(11) || driverHorizontal.getRawButtonPressed(11);
    }

    public boolean getDriverButton11Released() {
      return driverVertical.getRawButtonReleased(11) || driverHorizontal.getRawButtonReleased(11);
    }

    public boolean driverDeploy() {
      return getDriverButton6() || getDriverButton7() || getDriverButton8() || getDriverButton9()
            || getDriverButton10() || getDriverButton11();
    }

    public boolean driverDeployPressed() {
      return getDriverButton6Pressed() || getDriverButton7Pressed() || getDriverButton8Pressed() || getDriverButton9Pressed()
            || getDriverButton10Pressed() || getDriverButton11Pressed();
    }

    public boolean driverDeployReleased() {
      return getDriverButton6Released() || getDriverButton7Released() || getDriverButton8Released() || getDriverButton9Released()
            || getDriverButton10Released() || getDriverButton11Released();
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

    public boolean getLeftJoystickButtonPressed() {
      return gamepad.getRawButtonPressed(9);
    }

    public boolean getLeftJoystickButtonReleased() {
      return gamepad.getRawButtonReleased(9);
    }

    public boolean getRightJoystickButton() {
      return gamepad.getRawButton(10);
    }

    public boolean getRightJoystickButtonPressed() {
      return gamepad.getRawButtonPressed(10);
    }

    public boolean getRightJoystickButtonReleased() {
      return gamepad.getRawButtonReleased(10);
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

    public boolean getRightTrigger() {
      return (gamepad.getRawAxis(3) > 0.5 ? true : false);
    }

    public boolean getLeftTrigger() {
      return (gamepad.getRawAxis(2) > 0.5 ? true : false);
    }

    public int getPOV() {
      return gamepad.getPOV();
    }

    public boolean getPOVRight() {
      return (getPOV() == POV.RIGHT? true : false);
    }

    public boolean getPOVLeft() {
      return (getPOV() == POV.LEFT? true : false);
    }

    public boolean getPOVDown() {
      return (getPOV() == POV.DOWN? true : false);
    }

    public boolean getPOVUp() {
      return (getPOV() == POV.UP? true : false);
    }
  }
  
}
