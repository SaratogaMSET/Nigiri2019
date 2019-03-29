/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.MotionProfileCommand;

public class NearRocket extends CommandGroup {
  /**
   * Add your docs here.
   */
  public NearRocket(boolean isRightSide) { 
    if(isRightSide) {
      addSequential(new MotionProfileCommand("NearRocketRight", true));
    }
    else {
      addSequential(new MotionProfileCommand("NearRocketLeft", true));
    }
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
