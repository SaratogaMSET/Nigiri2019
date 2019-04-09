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
import frc.robot.commands.DeployHatchCommand;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.commands.PathTrigger;
import frc.robot.commands.semiauto.AutoIntakeHatch;
import frc.robot.commands.semiauto.HatchMid;
// import frc.robot.commands.EncoderTrigger;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

public class DoubleRocket extends CommandGroup {
  /**
   * Double hatch rocket auto.
   */
  public DoubleRocket(boolean isLeft) {
    addParallel(new PathTrigger(0.0, -22.98, -21.78, 150, new HatchMid()));
    addParallel(new PathTrigger(0.0, -17.83, -18.95, -180+30, new DeployHatchCommand()));
    addParallel(new PathTrigger(0.0, -4, -4, 179, new AutoIntakeHatch()));
    addParallel(new PathTrigger(0.0, -15.12, -15.79, 180, new Command(){
      @Override
      protected void initialize() {
        super.initialize();
        Robot.hatch.hatchDeployIn();
      }
      @Override
      // i think we should run a pure pursuit
      protected boolean isFinished() {
        return true;
      }
    }));
    addParallel(new PathTrigger(0.0, -15.12, -15.79, 180.0, new MoveLiftCommand(LiftPositions.LOW, 1.2)));


    // -20.65
    // right -21.30
    // gyro 13.12
    addSequential(new MotionProfileCommand("HAB1LxROCKLFxLOADLxROCKLF", 180.0));
  }
}
