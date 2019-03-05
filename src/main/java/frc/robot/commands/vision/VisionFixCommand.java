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
import jaci.pathfinder.Pathfinder;


public class VisionFixCommand extends FishyCommand {
  
  Timer time;
  GyroRotationalHoldCommand gyroHoldCommand;

  public VisionFixCommand() {
    time = new Timer();
    gyroHoldCommand = new GyroRotationalHoldCommand();
  }

  @Override
  protected String[] getLogFields() {
    return new String[] {"Angle"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
    gyroHoldCommand.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Activate vision target hold
    Robot.vision.readData();
    Double distance = Robot.vision.getDistance();
    Double angle = Robot.vision.getAngleDisplacement();

    if(angle == null || (distance != null && distance < 48.0)) {

    }
    else {
      gyroHoldCommand.setTargetAngle(Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle() + angle));
    }
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
    gyroHoldCommand.cancel();
    // SmartDashboard.putNumber("Left Encoder", Robot.drive.getLeftEncoder());
    // SmartDashboard.putNumber("Right Encoder", Robot.drive.getRightEncoder());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
