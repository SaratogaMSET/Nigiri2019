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

public class GyroStraightDistancePID extends Command {

  double hDistance;
  double vDistance;
  double targetHeading;
  double power;
  public GyroStraightDistancePID(double power, double targetHeading, double hDistance, double vDistance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.hDistance = hDistance;
    this.vDistance = vDistance;
    this.targetHeading = targetHeading;
    this.power = power;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double hGyroVal = 1.0/hDistance;
    double hGyroPID = Robot.drive.getGyroStraightPIDOutput(hGyroVal);
    double vGyroVal = 1.0/vDistance;
    double vGyroPID = Robot.drive.getGyroStraightPIDOutput(vGyroVal);
    double hDistPID = Robot.drive.getStraightPIDOutput(hDistance);
    double vDistPID = Robot.drive.getStraightPIDOutput(vDistance);
    double gyroError = Robot.gyro.getGyroAngle() - targetHeading;
    double gyroPID = Robot.gyro.getGyroStraightPIDOutput(gyroError);
    double output = gyroPID * (hGyroPID + vGyroPID);
    // double output = gyroPID;
    double driveOutput = power * (hDistPID + vDistPID);

    double right = driveOutput + output;
    double left = driveOutput - output;

    double max = Math.max(1, Math.max(Math.abs(right), Math.abs(left)));
    right /= max;
    left /= max;
    Robot.drive.rawDrive(left, right);
    
    SmartDashboard.putNumber("gyroPID output", gyroPID);
    SmartDashboard.putNumber("hGyroPID output", hGyroPID);
    SmartDashboard.putNumber("vGyroPID output", vGyroPID);
    SmartDashboard.putNumber("hDistPID output", hDistPID);
    SmartDashboard.putNumber("vDistPID output", vDistPID);
    SmartDashboard.putNumber("Drive Output", driveOutput);
    SmartDashboard.putNumber("Gyro Output", output);
    SmartDashboard.putNumber("Left power", left);
    SmartDashboard.putNumber("Right power", right);
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
