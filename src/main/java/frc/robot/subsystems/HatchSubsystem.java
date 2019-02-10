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

  public DoubleSolenoid hatchSol1;
  public DoubleSolenoid hatchSol2;
  public DigitalInput hatchAcquired;
  public boolean solOut;

  public HatchSubsystem() {
    hatchSol1 = new DoubleSolenoid(RobotMap.Hatch.HATCH_PISTON_1[0], RobotMap.Hatch.HATCH_PISTON_1[1]);
    hatchSol2 = new DoubleSolenoid(RobotMap.Hatch.HATCH_PISTON_2[0], RobotMap.Hatch.HATCH_PISTON_2[1]);
    hatchAcquired = new DigitalInput(RobotMap.Hatch.HATCH_SWITCH);
    solOut = false;
  }

  public void pistonsOut() {
    hatchSol1.set(DoubleSolenoid.Value.kForward);
    hatchSol2.set(DoubleSolenoid.Value.kForward);
    solOut = true;
  }

  public void pistonsIn() {
    hatchSol1.set(DoubleSolenoid.Value.kReverse);
    hatchSol2.set(DoubleSolenoid.Value.kReverse);
    solOut = false;
  }

  public boolean getPistonsOut() {
    return solOut;
  }

  public boolean getHatch() {
    return hatchAcquired.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
