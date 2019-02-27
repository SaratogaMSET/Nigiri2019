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

  public Driver driver;
  public Gamepad gamePad;
  public Operator operator;

  public OI() {
    driverHorizontal = new Joystick(RobotMap.JOYSTICK_DRIVE_HORIZONTAL);
    driverVertical = new Joystick(RobotMap.JOYSTICK_DRIVE_VERTICAL);
    operatorJoystick = new Joystick(RobotMap.OPERATOR_JOYSTICK);

  if(Robot.isGamepad) {
    gamepad = new Joystick(RobotMap.JOYSTICK_GAMEPAD);
  } else {
    buttonBoardLeft = new Joystick(RobotMap.BUTTON_BOARD_LEFT);
    buttonBoardRight = new Joystick(RobotMap.BUTTON_BOARD_RIGHT);
  }


    driver = new Driver();
    gamePad = new Gamepad();
    operator = new Operator();

    visionFixButton = new JoystickButton(driverHorizontal, 1); // driver rot stick trigger
  }

  public class Driver {
    private JoystickFilter verticalFilter = new JoystickFilter(0.0, 0.0, 1.0, Mode.LINEAR);
    private JoystickFilter horizontalFilter = new JoystickFilter(0.0, 0.0, 1.0, Mode.LINEAR);

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

    public boolean getDriverButton9() {
      return driverVertical.getRawButton(9) || driverHorizontal.getRawButton(9);
    }

    public boolean getDriverButton10() {
      return driverVertical.getRawButton(10) || driverHorizontal.getRawButton(10);
    }

    public boolean getDriverButton11() {
      return driverVertical.getRawButton(11) || driverHorizontal.getRawButton(11);
    }

    public boolean driverDeploy() {
      return getDriverButton6() || getDriverButton7() || getDriverButton8() || getDriverButton9()
            || getDriverButton10() || getDriverButton11();
    }
  }

  public class Operator{
    public boolean deployPressed() {
      if(Robot.isGamepad) {
        return gamePad.getBackButtonPressed();
      } else {
        return buttonBoardLeft.getRawButtonPressed(5);
      }
    }

    public boolean deploy() {
      if(Robot.isGamepad) {
        return gamePad.getBackButton();
      } else {
        return buttonBoardLeft.getRawButton(5);
      }
    }

    public boolean deployReleased() {
      if(Robot.isGamepad) {
        return gamePad.getBackButtonReleased();
      } else {
        return buttonBoardLeft.getRawButtonReleased(5);
      }
    }

    public boolean deployCargo() {
      if(Robot.isGamepad) {
        return gamePad.getBackButtonPressed() && gamePad.getLeftTrigger();
      } else {
        return buttonBoardRight.getRawButtonPressed(5);
      }
    }

    public boolean liftToLowCargo() {
      if(Robot.isGamepad) {
        return gamePad.getButtonAPressed();
      } else {
        return buttonBoardLeft.getRawButtonPressed(7);
      }
    }

    public boolean liftToLowHatch() {
      if(Robot.isGamepad) {
        return gamePad.getButtonAPressed();
      } else {
        return buttonBoardRight.getRawButtonPressed(7);
      }
    }

    public boolean liftToHatchMid() {
      if(Robot.isGamepad) {
        return gamePad.getButtonXPressed();
      } else {
        return buttonBoardLeft.getRawButtonPressed(6);
      }
    }

    public boolean liftToHatchHigh() {
      if(Robot.isGamepad) {
        return gamePad.getButtonYPressed();
      } else {
        return buttonBoardLeft.getRawButtonPressed(4);
      }
    }

    public boolean liftToCargoRocketLow() {
      if(Robot.isGamepad) {
        return gamePad.getButtonXPressed() && gamePad.getLeftTrigger();
      } else {
        return buttonBoardRight.getRawButtonPressed(3);
      }
    }

    public boolean liftToCargoRocketMid() {
      if(Robot.isGamepad) {
        return gamePad.getButtonYPressed() && gamePad.getLeftTrigger();
      } else {
        return buttonBoardRight.getRawButtonPressed(2);
      }
    }

    public boolean liftToCargoRocketHigh() {
      if(Robot.isGamepad) {
        return gamePad.getButtonBPressed() && gamePad.getLeftTrigger();
      } else {
        return buttonBoardRight.getRawButtonPressed(1);
      }
    }

    public boolean liftToCargoShip() {
      if(Robot.isGamepad) {
        return gamePad.getRightButtonPressed();
      } else {
        return buttonBoardRight.getRawButtonPressed(6);
      }
    }

    public boolean intakePressed() {
      if(Robot.isGamepad) {
        return gamePad.getLeftButtonPressed();
      } else {
        return buttonBoardRight.getRawButtonPressed(4);
      }
    }

    public boolean intake() {
      if(Robot.isGamepad) {
        return gamePad.getLeftButton();
      } else {
        return buttonBoardRight.getRawButton(4);
      }
    }

    public boolean intakeReleased() {
      if(Robot.isGamepad) {
        return gamePad.getLeftButtonReleased();
      } else {
        return buttonBoardRight.getRawButtonReleased(4);
      }
    }

    public boolean startClimbHAB2() {
      if(Robot.isGamepad) {
        return gamePad.getPOVLeft();
      } else {
        return buttonBoardLeft.getRawButtonPressed(1) && buttonBoardLeft.getRawButtonPressed(11);
      }
    }

    public boolean startClimbHAB3() {
      if(Robot.isGamepad) {
        return gamePad.getPOVRight();
      } else {
        return buttonBoardLeft.getRawButtonPressed(1) && buttonBoardLeft.getRawButtonPressed(10);
      }
    }

    public boolean bringDownJack() {
      if(Robot.isGamepad) {
        return gamePad.getPOVDown();
      } else {
        return buttonBoardLeft.getRawButtonPressed(2);
      }
    }

    public boolean bringUpJack() {
      if(Robot.isGamepad) {
        return gamePad.getPOVUp();
      } else {
        return buttonBoardLeft.getRawButtonPressed(3);
      }
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
