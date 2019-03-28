/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class GyroPIDCommand extends Command {

  Timer time;
  double timeout;

  double targetAngle;

  public GyroPIDCommand(double targetAngle, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.targetAngle = targetAngle;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time = new Timer();
    time.reset();
    time.start();

    Robot.gyro.gyroPIDController.setSetpoint(targetAngle);
    Robot.gyro.gyroPIDController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Gyro PID Output", Robot.gyro.getGyroPIDOutput());
    Robot.drive.rawDrive(Robot.gyro.getGyroPIDOutput(), -Robot.gyro.getGyroPIDOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.gyro.gyroPIDController.onTarget()) {
      return true;
    }
    else if (time.get() > timeout) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.gyro.gyroPIDController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
