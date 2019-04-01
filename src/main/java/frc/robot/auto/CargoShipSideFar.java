/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.Robot;

public class CargoShipSideFar extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CargoShipSideFar(boolean isRight) {
    if(isRight) {
      addSequential(new MotionProfileCommand("HAB1R-CR2", false));
    } else {
      addSequential(new MotionProfileCommand("HAB1L-CL2", false));
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
