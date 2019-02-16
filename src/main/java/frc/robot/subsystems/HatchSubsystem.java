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

/**
 * Add your docs here.
 */
public class HatchSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static enum HatchPositions {
    hatchIn,
    hatchOut,
    hatchDeploy
  }

  public Solenoid hatchSol;
  public Solenoid hatchDeploySol1;
  public Solenoid hatchDeploySol2;
  public DigitalInput hatchAcquired;
  public HatchPositions currentPosition;

  public HatchSubsystem() {
    hatchSol = new Solenoid(RobotMap.Hatch.HATCH_PISTON[0], RobotMap.Hatch.HATCH_PISTON[1]);
    hatchDeploySol1 = new Solenoid(RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[0], RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[1]);
    hatchDeploySol2 = new Solenoid(RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[0], RobotMap.Hatch.HATCH_DEPLOY_PISTON_2[1]);
    hatchAcquired = new DigitalInput(RobotMap.Hatch.HATCH_SWITCH);
    currentPosition = HatchPositions.hatchIn;
  }

  public void hatchOut() {
    hatchSol.set(true);
  }

  public void hatchIn() {
    hatchSol.set(false);
  }

  public void hatchDeploy() {
    hatchDeploySol1.set(true);
    hatchDeploySol2.set(true);
  }

  public void hatchDeployIn() {
    hatchDeploySol1.set(false);
    hatchDeploySol2.set(false);
  }

  public boolean getHatchAcquired() {
    return hatchAcquired.get();
  }

  public void moveHatchToPosition(HatchPositions pos) {
    switch(pos) {
      case hatchIn:
        hatchIn();
        break;
      case hatchOut:
        hatchOut();
        break;
      case hatchDeploy:
        hatchDeploy();
        break;
    }
  }

  public void setPosition(HatchPositions pos) {
    currentPosition = pos;
  }

  public HatchPositions getHatchPosition() {
    return currentPosition;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
