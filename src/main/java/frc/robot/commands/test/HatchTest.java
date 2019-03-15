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

public class HatchTest extends Command {
  public HatchTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.hatch.hatchDeployIn();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.oi.gamePad.getButtonYPressed()) {
      Robot.hatch.hatchOut();
    }
    if (Robot.oi.gamePad.getButtonAPressed()) {
      Robot.hatch.hatchIn();
    }
    if (Robot.oi.gamePad.getButtonBPressed()) {
      Robot.hatch.hatchDeploy();
    }
    if (Robot.oi.gamePad.getButtonXPressed()) {
      Robot.hatch.hatchDeployIn();
    }
    SmartDashboard.putBoolean("Hatch Position", Robot.hatch.getHatchPositionSol());
    SmartDashboard.putBoolean("Hatch Deploy", Robot.hatch.getHatchDeploySol());
    SmartDashboard.putString("Currently Running Diagnostic", "Hatch");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.gamePad.getBackButton();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.hatch.hatchIn();
    Robot.hatch.hatchDeployIn();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
