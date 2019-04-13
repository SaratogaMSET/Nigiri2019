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
import frc.robot.commands.GyroPIDCommand;
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
    
    addParallel(new PathTrigger(0.0, -1, -1, 179, new AutoIntakeHatch()));
    


    // -20.65
    // right -21.30
    // gyro 13.12
    addParallel(new PathTrigger(0, -20.44, -17.84, 120, new HatchMid()));
    addSequential(new MotionProfileCommand("DoubleRocketFast", 180.0));
    addSequential(new GyroPIDCommand(30, 10.0));
    addParallel(new DeployHatchCommand());
    addSequential(new MotionProfileCommand("DoubleRocketFast2", 180.0));
    addSequential(new GyroPIDCommand(-30, 10.0));
    addParallel(new Command(){
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
    });
    addParallel(new MoveLiftCommand(LiftPositions.LOW, 1.2));
    addParallel(new PathTrigger(0, -3.0, -3.0, 180, new GyroPIDCommand(180, 10.0)));
    addSequential(new MotionProfileCommand("DoubleRocketFast3", 180.0, -0.04));

  }
}
