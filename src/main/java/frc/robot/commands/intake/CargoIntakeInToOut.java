/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.util.Logging;

public class CargoIntakeInToOut extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoIntakeInToOut() {
    // if(Robot.isLogging) {
    //   String string = String.format("%.4f, CargoIntakeInToOut", Robot.time.get());
    //   Logging.print(string);
    // }
    addSequential(new SetMidStatePistons(false));
    addSequential(new SetIntakePistons(true));
    addSequential(new SetMidStatePistons(true));
  }
}