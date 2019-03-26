/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.RobotState;

public class DeployCommand extends Command {
  
  Timer timer;
  LiftPositions pos;
  boolean isFinished;
  double power;
  double timeout;

  public DeployCommand(LiftPositions liftPos, double power) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pos = liftPos;
    this.power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (pos == LiftPositions.CARGO_LOADING_STATION ||
     pos == LiftPositions.CARGO_ROCKET_LEVEL_TWO) {
      new DeployCargoCommand(-power).start();
    } else if (pos == LiftPositions.HATCH_MID || pos == LiftPositions.HATCH_HIGH) {
      new DeployHatchCommand().start();
    } else if (pos == LiftPositions.CARGO_ROCKET_LEVEL_ONE) {
      new SetIntakeRollers(false, 0.85, 0, 0.85).start();
    } else if (pos == LiftPositions.CARGO_ROCKET_LEVEL_THREE) {
      new DeployCargoCommand(-0.8).start();
    } else if(pos == LiftPositions.CARGO_SHIP) {
      new DeployCargoCommand(-0.5).start();
    } else {
      if(RobotState.hatchPositionState == HatchPositionState.HATCH_IN) {
        new DeployCargoCommand(-power).start();
      } else {
        new DeployHatchCommand().start();
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
