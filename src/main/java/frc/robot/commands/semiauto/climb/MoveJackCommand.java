/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.JackSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class MoveJackCommand extends Command {
  double position;
  double time;
  Timer timer;
  public MoveJackCommand(double pos,double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    position = pos;
    time = timeout;
    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   Robot.jack.setJackMotorMP(position);
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
    if(Math.abs(Robot.jack.getJackEncoder() - position) < 50 ){
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
