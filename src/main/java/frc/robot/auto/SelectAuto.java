/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.semiauto.AutoIntakeHatch;
import frc.robot.subsystems.AutoSelector.Side;
import frc.robot.subsystems.AutoSelector.Control;

public class SelectAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SelectAuto() {
    int autoNumber = Robot.autoSelector.getAutoPotNumber();
    Side side = Robot.autoSelector.getSide();
    Control control = Robot.autoSelector.getControl();

    if(control == Control.TELEOP) {
      addSequential(new AutoIntakeHatch());
    } else {
      switch(autoNumber) {
        case 1:
          if(side == Side.LEFT) {
            addSequential(Robot.backRocketLeft);
          } else {
            addSequential(Robot.backRocketRight);
          }
          break;
        case 2:
          if(side == Side.LEFT) {
            addSequential(Robot.nearRocketLeft);
          } else {
            addSequential(Robot.nearRocketRight);
          }
          break;
        case 3:

          break;
        case 4:

          break;
        case 5:

          break;
        case 6:

          break;
        case 7:

          break;
        case 8:

          break;
        case 9:

          break;
        case 10:

          break;
      }
    }
  }
}
