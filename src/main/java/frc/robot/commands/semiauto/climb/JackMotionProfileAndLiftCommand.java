/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.semiauto.climb;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.JackSubsystem;
import frc.robot.subsystems.LiftSubsystem;

public class JackMotionProfileAndLiftCommand extends Command {
  int jackHeight;
  int liftHeightEncoder;
  boolean isDown;
  double timeout;
  Timer time;
  int moveLiftVal;

  int initialLiftVal;

  // public static int JACK_AND_LIFT_OFFSET = 

  public JackMotionProfileAndLiftCommand(int jackHeight, int liftHeightEncoder, boolean isDown, double timeout) {
    // requires(Robot.jack);
    this.jackHeight = jackHeight;
    this.isDown = isDown;
    this.timeout = timeout;
    this.liftHeightEncoder = liftHeightEncoder;
    time = new Timer();
    initialLiftVal = Robot.lift.getRawEncoder();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putBoolean("Started", true);
    time.start();
    Robot.lift.setLiftMPHang();
    Robot.jack.setJackMPVals(isDown);
    Robot.jack.setJackMotorMP(jackHeight);
    initialLiftVal = Robot.lift.getRawEncoder();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    moveLiftVal = (int)(liftHeightEncoder - (Robot.jack.getJackEncoder()) * LiftSubsystem.LiftEncoderConstants.LIFT_TICKS_PER_JACK_TICK);
    SmartDashboard.putNumber("LIFT SETPOINT", moveLiftVal);
    if(moveLiftVal < (initialLiftVal - 300)) {
      Robot.lift.pidLift(Math.max(-20, moveLiftVal));
    }
    else {
      Robot.lift.setManualLift(0.0);
    }
    if(Robot.jack.isJackAtBottom()) {
      Robot.jack.setJackMotor(0.0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(time.get()>timeout || Robot.jack.isJackAtBottom() || (Math.abs(Robot.jack.getJackEncoder() - jackHeight)) < JackSubsystem.JackEncoderConstants.ABS_TOL){
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.setManualLift(0.0);
    Robot.jack.setJackMotor(0.0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
