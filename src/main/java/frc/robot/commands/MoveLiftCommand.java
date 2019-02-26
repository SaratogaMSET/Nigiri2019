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
import frc.robot.util.RobotState;

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
    current = RobotState.liftPosition;
    isFinished = false;
    onTarget = false;

    SmartDashboard.putBoolean("IN HERE", false);
    if(current == target) {
      SmartDashboard.putBoolean("IN HERE", true);
      isFinished = true;
    }
    time = new Timer();
    time.reset();
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.robotState.canRunLift()) {
      Robot.lift.moveLiftToPos(target);
      Robot.lift.setIsMoving(true);
      SmartDashboard.putString("Target Position", target.toString());
      // RobotState.liftPosition = LiftPositions.MOVING;
    } else {
      SmartDashboard.putBoolean("Here", true);
      isFinished = true;
    }

    SmartDashboard.putNumber("Time", time.get());
    SmartDashboard.putBoolean("is interrupted", false);
    SmartDashboard.putBoolean("onTarget", onTarget);
    SmartDashboard.putBoolean("isFinished", isFinished);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Robot.lift.withinTolerance(target)) {
      SmartDashboard.putString("Here", "Within Tol");
      onTarget = true;
      return true;
    } else if (time.get() > timeout) {
      SmartDashboard.putString("Here", "timeout");
      return true;
    }
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.setIsMoving(false);
    if(onTarget) {
      RobotState.liftPosition = target;
    }
    Robot.lift.setManualLift(0);
    SmartDashboard.putBoolean("onTarget", onTarget);
    SmartDashboard.putBoolean("isFinished", true);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    SmartDashboard.putBoolean("is interrupted", true);
    end();
  }
}
