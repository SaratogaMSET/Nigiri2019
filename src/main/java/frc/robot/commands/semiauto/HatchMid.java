/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.MoveHatchCommand;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

public class HatchMid extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HatchMid() {
    addParallel(new MoveHatchCommand(HatchPositionState.HATCH_OUT));
    addSequential(new MoveLiftCommand(LiftPositions.HATCH_MID, 1.2));
  }
}
