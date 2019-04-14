/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DeployHatchCommand;
import frc.robot.commands.GyroPIDCommand;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.semiauto.AutoIntakeHatch;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ForwardAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ForwardAuto(boolean close) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    // addSequential(new AutoIntakeHatch());
    if (close) {
      addSequential(new MotionProfileCommand("HABM-CF-close", false));
      /*addParallel(new DeployHatchCommand());
      addSequential(new MotionProfileCommand("CLF-LSL-Slow", true));
      addParallel(new Command(){
        @Override
        protected void initialize() {
          super.initialize();
          Robot.hatch.hatchDeployIn();
        }
        @Override
        protected boolean isFinished() {
          return true;
        }
      });
      addSequential(new MotionProfileCommand("LSL-CL1-Slow", true));
      */
    }
    else {
      addSequential(new MotionProfileCommand("HABM-CF", false));
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
