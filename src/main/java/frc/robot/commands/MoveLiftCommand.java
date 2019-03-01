/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.Robot;
import frc.robot.util.RobotState;

public class MoveLiftCommand extends Command {

  LiftPositions target;
  LiftPositions current;

  boolean isFinished;
  boolean onTarget;
  boolean goingUp;

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

    if(current == target) {
      isFinished = true;
    } else if (target == LiftPositions.CARGO_LOW && current == LiftPositions.HATCH_LOW) {
      isFinished = true;
    } else if (target == LiftPositions.HATCH_LOW && current == LiftPositions.CARGO_LOW) {
      isFinished = true;
    }else {
      goingUp = Robot.lift.goingUp(target, current);
    }
    time = new Timer();
    time.reset();
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.robotState.canRunLift() && !isFinished) {
      Robot.lift.moveLiftToPos(target);
      Robot.lift.setIsMoving(true);
      SmartDashboard.putString("Target Position", target.toString());
      RobotState.liftPosition = LiftPositions.MOVING;
      Robot.cargoIntake.runFrontRoller(goingUp, 0.8);
      Robot.cargoDeploy.runIntake(0.2);
    } else {
      isFinished = true;
    }

    SmartDashboard.putBoolean("is interrupted", false);
    SmartDashboard.putBoolean("onTarget", onTarget);
    SmartDashboard.putBoolean("isFinished", isFinished);
    SmartDashboard.putBoolean("Is Going Up", goingUp);
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
    
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.lift.setIsMoving(false);
    if(onTarget) {
      RobotState.liftPosition = target;
    }
    if(RobotState.cargoIntakeState != CargoIntakeState.OUT) {
      Robot.cargoIntake.runFrontRoller(goingUp, 0);
      Robot.cargoDeploy.runIntake(0);
    }
    
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
