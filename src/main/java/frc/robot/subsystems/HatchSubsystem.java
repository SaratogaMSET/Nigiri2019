/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.util.RobotState;

/**
 * Add your docs here.
 */
public class HatchSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static enum HatchState {
    hatchIn,
    hatchOut,
    hatchDeploy
  }

  public Solenoid hatchSol;
  public Solenoid hatchDeploySol1;
  public Solenoid hatchDeploySol2;
  public DigitalInput hatchAcquired;
  public HatchState currentState;

  public HatchSubsystem() {
    hatchSol = new Solenoid(RobotMap.Hatch.HATCH_PISTON[0], RobotMap.Hatch.HATCH_PISTON[1]);
    hatchDeploySol1 = new Solenoid(RobotMap.Hatch.HATCH_DEPLOY_PISTON[0], RobotMap.Hatch.HATCH_DEPLOY_PISTON[1]);
    //hatchAcquired = new DigitalInput(RobotMap.Hatch.HATCH_SWITCH);
    currentState = HatchState.hatchIn;
  }

  public void hatchOut() {
    hatchSol.set(true);
    RobotState.hatchState = HatchState.hatchOut;
    // changeHatchState();
  }

  public void hatchIn() {
    hatchSol.set(false);
    RobotState.hatchState = HatchState.hatchIn;
    // changeHatchState();
  }

  public void hatchDeploy() {
    hatchDeploySol1.set(true);
    // currentState = HatchState.hatchDeploy;
    // changeHatchState();
  }

  public void hatchDeployIn() {
    hatchDeploySol1.set(false);
    // currentState = HatchState.hatchOut;
    // changeHatchState();
  }

  public void changeHatchState() {
    if(!hatchSol.get()) {
      RobotState.hatchState = HatchState.hatchOut;
    } else {
      RobotState.hatchState = HatchState.hatchIn;
    }
  }

  public boolean getHatchAcquired() {
    return hatchAcquired.get();
  }

  public void moveHatch(HatchState pos) {
    switch(pos) {
      case hatchIn:
        hatchDeployIn();
        hatchIn();
        // changeHatchState();
        break;
      case hatchOut:
        hatchDeployIn();
        hatchOut();
        // changeHatchState();
        break;
      case hatchDeploy:
        hatchDeploy();
        break;
    }
  }

  public void setHatchState(HatchState pos) {
    currentState = pos;
  }

  public HatchState getHatchState() {
    return currentState;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
