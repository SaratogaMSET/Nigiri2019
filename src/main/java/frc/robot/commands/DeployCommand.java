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
import frc.robot.subsystems.HatchSubsystem.HatchState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

public class DeployCommand extends Command {
  
  Timer timer;
  LiftPositions pos;
  boolean isFinished;
  double power;
  double timeout;

  public DeployCommand(LiftPositions liftPos, double power, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.pos = liftPos;
    this.power = power;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();

    isFinished = false;

    if (pos == LiftPositions.CARGO_LOW || pos == LiftPositions.CARGO_SHIP || pos == LiftPositions.CARGO_LOADING_STATION ||
    pos == LiftPositions.CARGO_ROCKET_LEVEL_ONE || pos == LiftPositions.CARGO_ROCKET_LEVEL_TWO || pos == LiftPositions.CARGO_ROCKET_LEVEL_THREE) {
      new DeployCargoCommand(-power).start();
    } else if (pos == LiftPositions.HATCH_LOW || pos == LiftPositions.HATCH_MID || pos == LiftPositions.HATCH_HIGH) {
      new DeployHatchCommand().start();
    } else {
      isFinished = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (timer.get() > timeout) {
      isFinished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    timer.stop();
    if (Robot.hatch.getHatchState() == HatchState.hatchDeploy) {
      new DeployHatchCommand().start();
    }
    else {
      new DeployCargoCommand(0).start();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
