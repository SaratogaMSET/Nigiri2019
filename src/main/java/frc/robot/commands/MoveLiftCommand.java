/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.Robot;

public class MoveLiftCommand extends Command {

  LiftPositions target;
  LiftPositions current;

  boolean isFinished;
  boolean onTarget;

  Timer time;

  double timeout;

  public MoveLiftCommand(LiftPositions target, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.target = target;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    current = Robot.lift.getLiftPosition();
    isFinished = false;
    onTarget = false;

    if(current == target || Robot.lift.getIsMoving()) {
      isFinished = true;
      // stall lift command
    }
    if(!Robot.cargoIntake.solOut()) {
      end();
    }

    time = new Timer();
    Robot.lift.setIsMoving(true);
    Robot.lift.moveLiftToPos(target);
    time.reset();
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.lift.moveLiftToPos(target);

    

    SmartDashboard.putBoolean("onTarget", onTarget);
    SmartDashboard.putBoolean("isFinished", isFinished);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.lift.withinTolerance(target)) {
      onTarget = true;
      return true;
    } else if (time.get() > timeout) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.setIsMoving(false);
    if(onTarget) {
      Robot.lift.setPosition(target);
    }
    SmartDashboard.putBoolean("onTarget", onTarget);
    SmartDashboard.putBoolean("isFinished", true);

    // start stall lift
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
