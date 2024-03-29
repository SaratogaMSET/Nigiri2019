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
import frc.robot.subsystems.CargoDeploySubsystem.CargoDeployMotorState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeMotorState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.Robot;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.util.Logging;
import frc.robot.RobotState;

public class MoveLiftCommand extends Command {

  LiftPositions target;
  LiftPositions current;

  boolean isFinished;
  boolean onTarget;
  boolean goingUp;
  boolean goingToBottom;
  boolean isTargetCargoState;

  Timer time;

  double timeout;
  double intakePower;

  public MoveLiftCommand(LiftPositions target, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.lift);
    this.target = target;
    this.timeout = timeout;
  }

  protected String[] getLogFields() {
    // velocities for MP
    return new String[] {"vel", "time"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotState.isManualLift = false;
    SmartDashboard.putBoolean("isFInishedLIFT",false);

    RobotState.lastLiftTarget = target;

    if(RobotState.liftPosition != LiftPositions.LOW && target != LiftPositions.LOW) {
      Robot.compressor.stop();
    }

    current = RobotState.liftPosition;
    isFinished = false;
    onTarget = false;
    goingToBottom = false;
    isTargetCargoState = RobotState.isLiftCargoState(target);

    if(current == target) {
      isFinished = true;
    } else if(target == LiftPositions.LOW) {
      goingToBottom = true;
    } else {
      goingUp = Robot.lift.goingUp(target, current);
    }
    
    time = new Timer();
    time.reset();
    time.start();

    if(Robot.isLogging) {
      String string = String.format("%.4f, MoveLiftCommand Start, Current: %.2f, Target: %s", Robot.time.get(), Robot.lift.getDistance(), target.toString());
      Logging.print(string);
    }
    RobotState.isRunningLiftCommand = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // log("vel",Robot.lift.getVel());
    // log("time",time.getFPGATimestamp());
    // logger.write();
    if(RobotState.canRunLift() && !isFinished) {

      Robot.lift.moveLiftToPos(target);
      SmartDashboard.putString("Target", target.toString());
      if(isTargetCargoState) {
        if(goingUp && RobotState.runIntakesWhileLifting()) {
          if(RobotState.intakeMotorState != CargoIntakeMotorState.TOP_BAR_ONLY) {
            new SetIntakeRollers(true, 0.6, 0, 0.2).start();
          }
        } else {
          if(RobotState.intakeMotorState != CargoIntakeMotorState.NONE) {
            new SetIntakeRollers(true, 0, 0, 0.2).start();
          }
        }
      }
      if(target == LiftPositions.CARGO_SHIP) {
        if(Robot.lift.getRawEncoder() > Robot.lift.getLiftPositionEncoders(LiftPositions.CARGO_ROCKET_LEVEL_ONE)) {
          // new ChangeIntakeState(CargoIntakePositionState.IN).start();
        }
      }
    } else {
      if(!RobotState.canRunLift()) {
        Robot.lift.setManualLift(0);
        new LiftWhenSafe(target, 2).start();
      }
      isFinished = true;
    }
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
    RobotState.isRunningLiftCommand = false;
    // logger.drain();
    // logger.flush();
    if(RobotState.intakeMotorState == CargoIntakeMotorState.TOP_BAR_ONLY && !Robot.oi.gamePad.getLeftButton()) {
      new SetIntakeRollers(true, 0).start();
    }
    if(RobotState.liftPosition != LiftPositions.LOW) {
      new SetIntakeRollers(true, 0).start();
    }
    
    if(!RobotState.canRunLift()) {
      isFinished = true;
      new LiftWhenSafe(target, 2).start();
    }

    if(onTarget) {
      if(target == LiftPositions.CARGO_LOADING_STATION || target == LiftPositions.CARGO_SHIP) {
        if(RobotState.cargoIntakeState != CargoIntakePositionState.IN) {
          // new ChangeIntakeState(CargoIntakePositionState.IN).start();
        }
      }
    }

    if(RobotState.liftPosition == LiftPositions.LOW) {
      Robot.compressor.start();
    }

    // if(RobotState.liftPosition == LiftPositions.CLIMB_HAB_THREE_TOL || RobotState.liftPosition == LiftPositions.CLIMB_HAB_TWO_TOL) {
    //   Robot.isClimb = true;
    // }

    if(Robot.isLogging) {
      String string = String.format("%.4f, MoveLiftCommand End, Current: %.2f, Target: %s", 
      Robot.time.get(), Robot.lift.getDistance(), target.toString());
      Logging.print(string);
    }
    SmartDashboard.putBoolean("isFInishedLIFT",true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
