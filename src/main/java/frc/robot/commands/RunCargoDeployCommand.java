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
import frc.robot.util.Logging;

public class RunCargoDeployCommand extends Command {
  public RunCargoDeployCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Robot.isLogging) {
      String string = String.format("%.4f, RunCargoDeployCommand", Robot.time.get());
      Logging.print(string);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.cargoDeploy.runIntake(Robot.oi.driverVertical.getY());
    if(Robot.oi.driverVertical.getTriggerPressed()) {
      Robot.cargoDeploy.anglePistons();
    } 
    // SmartDashboard.putBoolean("Acquired Cargo", Robot.cargoDeploy.hasCargo());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.driverVertical.getRawButton(6);
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
