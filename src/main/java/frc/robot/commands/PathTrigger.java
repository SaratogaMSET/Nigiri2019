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
import frc.robot.util.FishyMath;

public class PathTrigger extends Command {
  double leftDistanceTarget;
  double rightDistanceTarget;
  double gyroHeadingTarget;
  double timeTarget;

  Command com;
  boolean isTriggered = false;
  
  public PathTrigger(double matchTimeTarget, double leftEncoderDistance, double rightEncoderDistance, double gyroHeadingTarget, Command command) {
    // this.encoderVal = encoderVal;
    this.com = command;
    this.leftDistanceTarget = leftEncoderDistance;
    this.rightDistanceTarget = rightEncoderDistance;
    this.gyroHeadingTarget = gyroHeadingTarget;
    this.timeTarget = matchTimeTarget;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(!isTriggered && 
      Math.abs(Robot.drive.getLeftEncoderDistance() - leftDistanceTarget) <= 0.5 &&
      Math.abs(Robot.drive.getRightEncoderDistance() - rightDistanceTarget) <= 0.5 &&
      Math.abs(FishyMath.boundThetaNeg180to180(Robot.gyro.getGyroAngle() - gyroHeadingTarget)) <= 10.0 &&
      Math.abs(Timer.getMatchTime() - timeTarget) < 15.0
      ){

      com.start();
      isTriggered = true;

    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.autoControl || (isTriggered && !com.isRunning() && com.isCompleted());
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
