/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.MoveHatchCommand;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.*;
import frc.robot.Robot;
import frc.robot.util.Logging;

public class DefenseModeCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DefenseModeCommand() {
    if(Robot.isLogging) {
      String string = String.format("%.4f, DefenseModeCommand", Robot.time.get());
      Logging.print(string);
    }
    addSequential(new MoveHatchCommand(HatchPositionState.HATCH_IN));
    addSequential(new MoveLiftCommand(LiftPositions.LOW, 2));
    addSequential(new ChangeIntakeState(CargoIntakePositionState.IN));
  }
}
