/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.*;
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

  public static enum HatchPositionState {
    HATCH_IN,
    HATCH_OUT
  }

  public static enum HatchDeployState {
    DEPLOY,
    HOLD
  }

  public static enum HatchGamePiece {
    NO_HATCH,
    HAVE_HATCH
  }  

  public Solenoid hatchSol;
  public Solenoid hatchDeploySol;
  public DigitalInput hatchAcquired;

  public HatchSubsystem() {
    hatchSol = new Solenoid(RobotMap.Hatch.HATCH_PISTON[0], RobotMap.Hatch.HATCH_PISTON[1]);
    hatchDeploySol = new Solenoid(RobotMap.Hatch.HATCH_DEPLOY_PISTON[0], RobotMap.Hatch.HATCH_DEPLOY_PISTON[1]);
    //hatchAcquired = new DigitalInput(RobotMap.Hatch.HATCH_SWITCH);
  }

  public void hatchOut() {
    hatchSol.set(true);
  }

  public void hatchIn() {
    hatchSol.set(false);
  }

  public void hatchDeploy() {
    hatchDeploySol.set(true);
  }

  public void hatchDeployIn() {
    hatchDeploySol.set(false);
  }

  public void updateHatchPositionState() {
    if(hatchSol.get()) {
      RobotState.hatchPositionState = HatchPositionState.HATCH_OUT;
    } else {
      RobotState.hatchPositionState = HatchPositionState.HATCH_IN;
    }
  }

  public void updateHatchDeployState() {
    if(hatchDeploySol.get()) {
      RobotState.hatchDeployState = HatchDeployState.DEPLOY;
    } else {
      RobotState.hatchDeployState = HatchDeployState.HOLD;
    }
  }

  public void checkHatchStateValid() {
    HatchPositionState posState = RobotState.hatchPositionState;
    HatchDeployState deployState = RobotState.hatchDeployState;
    if(posState == HatchPositionState.HATCH_IN) {
      if(deployState == HatchDeployState.DEPLOY) {
        SmartDashboard.putString("HATCH CHECK", "INVALID");
      }
    } else {
      SmartDashboard.putString("HATCH CHECK", "valid");
    }
    updateHatchDeployState();
    updateHatchPositionState();
  }

  public boolean getHatchAcquired() {
    return hatchAcquired.get();
  }

  public void moveHatch(HatchPositionState pos) {
    switch(pos) {
      case HATCH_IN:
        hatchDeployIn();
        hatchIn();
        break;
      case HATCH_OUT:
        hatchDeployIn();
        hatchOut();
        break;
    }
  }

  public void checkState() {
    if(RobotState.hatchPositionState == HatchPositionState.HATCH_IN) {
      if(RobotState.hatchDeployState == HatchDeployState.DEPLOY) {
        moveHatch(HatchPositionState.HATCH_IN);
      }
    }
    updateHatchDeployState();
    updateHatchPositionState();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
