/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
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
    } else {
      addParallel(new AutoIntakeHatch());
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
          Robot.secondLeg = new NearRocketToLoadingStation();
          break;
        case 3:
          addSequential(Robot.closeCargoShip);
          break;
        case 4:
          addSequential(Robot.cargoShipAuto);
          break;
        case 5:
          if(side == Side.LEFT) {
            addSequential(Robot.cargoSideLeft);
          } else {
            addSequential(Robot.cargoSideRight);
          }
          break;
        case 6:
          addSequential(Robot.testDTMaxVA);
          break;
        case 7:
          addSequential(Robot.testTalonVel);
          break;
        case 8:

          break;
        case 9:

          break;
        case 10:

          break;
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
}
