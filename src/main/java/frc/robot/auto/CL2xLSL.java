/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.GyroPIDCommand;
import frc.robot.commands.MotionProfileCommand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CL2xLSL extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CL2xLSL() {
    addSequential(new GyroPIDCommand(0, 1.5));
    addSequential(new MotionProfileCommand("CL2-LSL", false));
    addSequential(new Command() {
      @Override
      protected void initialize() {
          Robot.switchAutoToTeleop();
      }
      @Override
      protected boolean isFinished() {
        return Robot.autoControl == false;
      }
    });
  }
}
