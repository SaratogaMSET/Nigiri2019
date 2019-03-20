/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import edu.wpi.first.wpilibj.Timer;

public class DeployClimbForks extends Command {
  boolean isOut;
  double time;
  Timer timer;
  public DeployClimbForks(boolean isOut, double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.isOut = isOut;
    timer = new Timer();
    this.time = time;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.jack.releaseForks(isOut);
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(timer.get() > time){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    timer.stop();
    timer.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
