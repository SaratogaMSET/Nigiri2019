/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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

  public DoubleSolenoid hatchSol;
  public DoubleSolenoid hatchDeploySol1;
  public DoubleSolenoid hatchDeploySol2;
  public DigitalInput hatchAcquired;
  public HatchPositions currentPosition;

  public HatchSubsystem() {
    hatchSol = new DoubleSolenoid(RobotMap.Hatch.HATCH_PISTON[0], RobotMap.Hatch.HATCH_PISTON[1], RobotMap.Hatch.HATCH_PISTON[2]);
    hatchDeploySol1 = new DoubleSolenoid(RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[0], RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[1], RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[3]);
    hatchDeploySol2 = new DoubleSolenoid(RobotMap.Hatch.HATCH_DEPLOY_PISTON_1[0], RobotMap.Hatch.HATCH_DEPLOY_PISTON_2[1], RobotMap.Hatch.HATCH_DEPLOY_PISTON_2[2]);
    hatchAcquired = new DigitalInput(RobotMap.Hatch.HATCH_SWITCH);
    currentPosition = HatchPositions.hatchIn;
  }

  public void hatchOut() {
    hatchSol.set(DoubleSolenoid.Value.kForward);
  }

  public void hatchIn() {
    hatchSol.set(DoubleSolenoid.Value.kReverse);
  }

  public void hatchDeploy() {
    hatchDeploySol1.set(DoubleSolenoid.Value.kForward);
    hatchDeploySol2.set(DoubleSolenoid.Value.kForward);
  }

  public void hatchDeployIn() {
    hatchDeploySol1.set(DoubleSolenoid.Value.kReverse);
    hatchDeploySol2.set(DoubleSolenoid.Value.kReverse);
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
