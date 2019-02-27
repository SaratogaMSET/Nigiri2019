/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.intake.ExtakeForTime;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

public class LiftToPositionAndShoot extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LiftToPositionAndShoot(LiftPositions pos) {
    addSequential(new MoveLiftCommand(pos, 2));
    addSequential(new ExtakeForTime(1, 0.5));
  }
}
