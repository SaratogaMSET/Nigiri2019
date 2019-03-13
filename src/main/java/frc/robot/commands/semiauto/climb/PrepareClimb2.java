/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LedPatternCommand;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
public class PrepareClimb2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PrepareClimb2() {

    addSequential(new MoveLiftCommand(LiftPositions.PREP_CLIMB_1,2));
    addSequential(new DeployClimbForks(true,0.1));
    addSequential(new MoveLiftCommand(LiftPositions.PREP_CLIMB_2,2));
    addSequential(new DeployClimbForks(false,0.1));
    addSequential(new MoveLiftCommand(LiftPositions.CLIMB_HAB_TWO_TOL,2));
  }
}
