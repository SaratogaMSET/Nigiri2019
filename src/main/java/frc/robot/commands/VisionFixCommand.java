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

public class VisionFixCommand extends FishyCommand {
  
  Timer time;
  int numUpdates = 0;

  public VisionFixCommand() {
    super(Robot.drive, Robot.gyro);
    time = new Timer();
  }

  @Override
  protected String[] getLogFields() {
    return new String[] {"Angle"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
    Robot.drive.changeBrakeCoast(false);
    numUpdates = 0;
    Robot.gyro.gyroPIDController.setSetpoint(Robot.gyro.getGyroAngle());
    Robot.gyro.gyroPIDController.enable(); // doesn't actuate motors, only writes to gyroPIDOutput variables
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Activate vision target hold
    Robot.vision.readData();
    Double angle = Robot.vision.getAngleDisplacement();
    log("Angle", angle);
    if(numUpdates < 3) {
      if(angle != null) {
        SmartDashboard.putNumber("VISION CA", angle);
        Robot.gyro.gyroPIDController.setSetpoint(Robot.gyro.getGyroAngle() + angle);
        numUpdates += 1;
      }
      else {
        numUpdates = 0;
        Robot.drive.driveFwdRotate(Robot.oi.driver.getDriverVertical(), Robot.oi.driver.getDriverHorizontal());
        return;
      }
    }
    logger.write();
    Robot.drive.driveFwdRotate(Robot.oi.driver.getDriverVertical(), Robot.gyro.getGyroPIDOutput());
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Let Command be canceled manually
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.gyro.gyroPIDController.disable();
    // SmartDashboard.putNumber("Left Encoder", Robot.drive.getLeftEncoder());
    // SmartDashboard.putNumber("Right Encoder", Robot.drive.getRightEncoder());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
