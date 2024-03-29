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

public class LiftTest extends Command {

  private int currentlyRunningMotor;
  private double slow;
  public LiftTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.lift.resetEncoder();
    Robot.lift.setManualLift(0);
    Robot.lift.setPercentOutput();
    currentlyRunningMotor = 0;
    slow = 2.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.oi.gamePad.getButtonBPressed()) {
      currentlyRunningMotor = 1;
    } else if(Robot.oi.gamePad.getButtonYPressed()) {
      currentlyRunningMotor = 2;
    } else if(Robot.oi.gamePad.getButtonXPressed()) {
      currentlyRunningMotor = 3;
    }

    if(Robot.oi.gamePad.getPOVUp()) {
      slow = 1.0;
    } else if (Robot.oi.gamePad.getPOVDown()) {
      slow = 2.0;
    }

    if(Robot.oi.gamePad.getPOVRight()) {
      Robot.lift.setPercentOutput();
    } else if (Robot.oi.gamePad.getPOVLeft()) {
      Robot.lift.setFollowers();
    }

    
    Robot.lift.setLiftMotor(currentlyRunningMotor, Robot.oi.gamePad.getLeftJoystickY()/slow);

    SmartDashboard.putNumber("Currently Running Motor ID", Robot.lift.getLiftMotorID(currentlyRunningMotor));
    SmartDashboard.putNumber("Lift Encoder", Robot.lift.getRawEncoder());
    SmartDashboard.putBoolean("Slow", (slow == 1? false : true));
    // SmartDashboard.putBoolean("Top Hal", Robot.lift.getTopHal());
    SmartDashboard.putBoolean("Bottom Hal", Robot.lift.getBottomHal());
    SmartDashboard.putString("Currently Running Diagnostic", "Lift");


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.gamePad.getBackButton();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.setFollowers();
    Robot.lift.setManualLift(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
