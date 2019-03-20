/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto.climb;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.RobotState;

public class RunLiftTillZero extends Command {
  double power;
  boolean isFinished;
  double timeout = 0.25;
  Timer timer;
  public RunLiftTillZero(double power) {
    this.power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isFinished = false;
    timer = new Timer();
    timer.reset();
    timer.start();
    if(RobotState.liftPosition == LiftPositions.LOW) {
      Robot.lift.setManualLift(-power);
    } else {
      isFinished = true;
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.lift.getBottomHal() || timer.get() > timeout || isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.setManualLift(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
