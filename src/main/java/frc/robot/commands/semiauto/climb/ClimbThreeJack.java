/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.subsystems.JackSubsystem;
import frc.robot.subsystems.LiftSubsystem.LiftEncoderConstants;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

public class ClimbThreeJack extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ClimbThreeJack() {
    // addSequential(new MoveLiftCommand(LiftPositions.CLIMB_HAB_THREE, 2));
    addSequential(new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstants.DOWN_STATE_LEVEL_3, LiftEncoderConstants.CLIMB_HAB_THREE, true, 1000));

  }
}
