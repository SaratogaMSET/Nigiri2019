/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.semiauto.climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.JackSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class JackMotionProfileAndLiftCommand extends Command {
  int jackHeight;
  boolean isDown;
  double timeout;
  Timer time;
  int moveLiftVal;
  public JackMotionProfileAndLiftCommand(int jackHeight, boolean isDown, double timeout) {
    // requires(Robot.jack);
    this.jackHeight = jackHeight;
    this.isDown = isDown;
    this.timeout = timeout;
    time = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putBoolean("Started", true);
    time.start();
    Robot.jack.setJackMPVals(isDown);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    moveLiftVal = (int)(LiftSubsystem.LiftTargetEncoderTicks.CLIMB_HAB_THREE - Robot.jack.getJackEncoder() * LiftSubsystem.LiftTargetEncoderTicks.LIFT_TICKS_PER_JACK_TICK);
    SmartDashboard.putNumber("LIFT SETPOINT", moveLiftVal);
    Robot.lift.pidLift(moveLiftVal);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(time.get()>timeout || Robot.jack.isJackAtBottom() || (Math.abs(Robot.jack.getJackEncoder() - jackHeight)) < JackSubsystem.JackEncoderConstatns.ABS_TOL){
      return true;
    }
    return false;
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
