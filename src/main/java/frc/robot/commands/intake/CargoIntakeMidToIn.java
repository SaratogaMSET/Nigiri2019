/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoIntakeMidToIn extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoIntakeMidToIn() {
    addSequential(new SetIntakePistons(true));
    addSequential(new SetMidStatePistons(true));
    addSequential(new SetIntakePistons(false));
    addSequential(new UpdateIntakeStateCommand());
  }
}
