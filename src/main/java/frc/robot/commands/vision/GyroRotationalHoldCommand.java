/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.FishyCommand;
import frc.robot.util.FishyMath;

/**
 * Takes driver input for forward and freezes input on rotational axis of robot, using a Gyro PID controller instead.
 */
public class GyroRotationalHoldCommand extends FishyCommand {
  
  Timer time;
  double targetAngle;

  public GyroRotationalHoldCommand() {
    time = new Timer();
  }

  @Override
  protected String[] getLogFields() {
    return new String[] {"Angle"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // targetAngle = Robot.gyro.getGyroAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
    Robot.gyro.driverGyroPID.setSetpoint(Robot.gyro.getGyroAngle());
    Robot.gyro.gyroPIDController.disable();
    Robot.gyro.gyroPIDOutput = 0.0;
    // SmartDashboard.putNumber("Left Encoder", Robot.drive.getLeftEncoder());
    // SmartDashboard.putNumber("Right Encoder", Robot.drive.getRightEncoder());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }

  public void setTargetAngle(double angle) {
    angle = FishyMath.boundThetaNeg180to180(angle);
    targetAngle = angle;
    Robot.gyro.gyroPIDController.setSetpoint(targetAngle);
    Robot.gyro.gyroPIDController.enable();
  }
}
