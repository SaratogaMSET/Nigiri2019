/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoIntakeInToMid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoIntakeInToMid() {
    addSequential(new SetMidStatePistons(true,2));
    addSequential(new SetIntakePistons(true, 2));
    addSequential(new SetMidStatePistons(false, 2));
    addSequential(new SetIntakePistons(false, 2));
  }
}