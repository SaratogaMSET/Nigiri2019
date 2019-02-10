/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class MotionProfileCommand extends Command {
  
  String pathName;
  double p;
  double i;
  double d;
  double v;
  double turnVal;
  boolean revPath;

  public MotionProfileCommand(String pathName, double p, double i, double d, double v, double turnVal, boolean revPath) {
    this.pathName = pathName;
    this.p = p;
    this.i = i;
    this.d = d;
    this.v = v;
    this.turnVal = turnVal;
    this.revPath = revPath;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.runPath(pathName, p, i, d, v, turnVal, revPath);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.drive.isMPFinish();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.stopMP();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
