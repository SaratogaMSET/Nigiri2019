/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DeployCargoCommand extends Command {

  boolean isFinished;

  Timer time;
  
  double power;
  double timeout;

  public DeployCargoCommand(double power, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.power = power;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time = new Timer();
    time.reset();
    time.start();

    isFinished = false;

    Robot.cargoDeploy.runIntake(power);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (time.get() > timeout) {
      isFinished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    time.stop();
    Robot.cargoDeploy.runIntake(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
