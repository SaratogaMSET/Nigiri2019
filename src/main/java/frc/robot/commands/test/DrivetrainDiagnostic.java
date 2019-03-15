/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class DrivetrainDiagnostic extends Command {

  int motorNum;
  public DrivetrainDiagnostic() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    motorNum = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.oi.gamePad.getButtonAPressed()) {
      increaseMotorNum();
    }

    Robot.drive.testMotor(motorNum, Robot.oi.gamePad.getRightJoystickY());
    
    SmartDashboard.putNumber("Drive Left Encoder", Robot.drive.getRawLeftEncoder());
    SmartDashboard.putNumber("Drive Right Encoder", Robot.drive.getRawRightEncoder());
    SmartDashboard.putString("Currently Running Diagnostic", "Drivetrain");

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.gamePad.getBackButtonPressed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.rawDrive(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  private void increaseMotorNum() {
    if(motorNum < 5) {
      motorNum ++;
    } else {
      motorNum = 0;
    }
  }
}
