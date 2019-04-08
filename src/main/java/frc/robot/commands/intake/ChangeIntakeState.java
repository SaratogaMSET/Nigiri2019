/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.RobotState;
import frc.robot.util.Logging;
import edu.wpi.first.wpilibj.Timer;

public class ChangeIntakeState extends Command {

  CargoIntakePositionState targetState;
  HatchPositionState targetHatchState;
  Timer time;
  double targetTime;
  public ChangeIntakeState(CargoIntakePositionState targetState) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.targetState = targetState;
    targetHatchState = HatchPositionState.HATCH_IN;
    time = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
    HatchPositionState currentHatchState = RobotState.hatchPositionState;

    if(currentHatchState == targetHatchState){
      targetTime = 0;
    } else {
      targetTime = 0;
      Robot.hatch.moveHatch(HatchPositionState.HATCH_IN);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return time.get() > targetTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    time.stop();
    time.reset();

    CargoIntakePositionState currentState = RobotState.cargoIntakeState;

    if(Robot.isLogging) {
      String string = String.format("%.4f, ChangeIntakeState, Current: %s, Target: %s", Robot.time.get(), currentState.toString(), targetState.toString());
      Logging.print(string);
    }
    // RobotState.cargoIntakeTargetState = targetState;
    
    if(targetState == currentState) {
      // it be done;
    } else if(targetState == CargoIntakePositionState.IN && RobotState.canBringIntakeIn()) {
      if(currentState == CargoIntakePositionState.OUT) {
        new CargoIntakeOutToIn().start();
      } else if(currentState == CargoIntakePositionState.MID) {
        new CargoIntakeMidToIn().start();
      } else if(currentState == CargoIntakePositionState.MOVING) {
        new CargoIntakeMidToIn().start();
      }
    } else if(targetState == CargoIntakePositionState.OUT) {
      if(currentState == CargoIntakePositionState.IN) {
        new CargoIntakeInToOut().start();
      } else if(currentState == CargoIntakePositionState.MID) {
        new CargoIntakeMidToOut().start();
      } else if(currentState == CargoIntakePositionState.MOVING) {
        new CargoIntakeInToOut().start();
      }
    } else if(targetState == CargoIntakePositionState.MID) {
      if(currentState == CargoIntakePositionState.IN) {
        new CargoIntakeInToMid().start();
      } else if(currentState == CargoIntakePositionState.OUT) {
        new CargoIntakeOutToMid().start();
      } else if(currentState == CargoIntakePositionState.MOVING) {
        new CargoIntakeInToMid().start();
      }
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
