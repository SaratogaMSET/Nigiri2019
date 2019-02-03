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

public class VisionFixCommand extends Command {
  
  Timer time;

  public VisionFixCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    time = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
    Robot.drive.changeBrakeCoast(false);
    Robot.drive.rawDrive(0, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.gyro.gyroPIDController.enable();
    // Activate vision target hold
    Robot.vision.readData();
    Double angle = Robot.vision.getAngleDisplacement();
    if(angle != null) {
        Robot.gyro.gyroPIDController.setSetpoint(Robot.gyro.gyro.getAngle() + angle);
        double[] drivePower = Robot.oi.driver.getArcadePower();
        drivePower[0] += Robot.gyro.getGyroPIDOutput();
        drivePower[1] -= Robot.gyro.getGyroPIDOutput();
        double normalizer = Math.max(1, Math.max(Math.abs(drivePower[0]), Math.abs(drivePower[1])));
        drivePower[0] /= normalizer;
        drivePower[1] /= normalizer;
        Robot.drive.rawDrive(drivePower[0], drivePower[1]);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.gyro.gyroPIDController.disable();
    Robot.drive.rawDrive(0, 0);
    SmartDashboard.putNumber("Left Encoder", Robot.drive.getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", Robot.drive.getRightEncoder());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
