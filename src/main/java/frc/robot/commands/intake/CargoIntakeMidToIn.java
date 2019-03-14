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

public class CargoIntakeMidToIn extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoIntakeMidToIn() {
    if(Robot.isLogging) {
      String string = String.format("%.4f, CargoIntakeMidToIn", Robot.time.get());
      Logging.print(string);
    }
    addSequential(new SetIntakePistons(true));
    addSequential(new SetMidStatePistons(false));
    addSequential(new SetIntakePistons(false));
  }
}
