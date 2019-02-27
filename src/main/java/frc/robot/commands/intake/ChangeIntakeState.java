/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;
import frc.robot.subsystems.HatchSubsystem.HatchState;
import frc.robot.util.RobotState;
import edu.wpi.first.wpilibj.Timer;

public class ChangeIntakeState extends Command {

  CargoIntakeState targetState;
  HatchState targetHatchState;
  Timer time;
  double targetTime;
  public ChangeIntakeState(CargoIntakeState targetState) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.targetState = targetState;
    targetHatchState = HatchState.hatchIn;
    time = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
    CargoIntakeState currentState = RobotState.cargoIntakeState;
    HatchState currentHatchState = RobotState.hatchState;

    if(currentHatchState == targetHatchState){
      targetTime = 0;
    } else {
      targetTime = 0.5;
      Robot.hatch.moveHatch(HatchState.hatchIn);
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
    CargoIntakeState currentState = RobotState.cargoIntakeState;
    
    if(targetState == currentState) {
      // it be done;
    } else if(targetState == CargoIntakeState.IN && Robot.robotState.canBringIntakeIn()) {
      if(currentState == CargoIntakeState.OUT) {
        new CargoIntakeOutToIn().start();
      } else if(currentState == CargoIntakeState.MID) {
        new CargoIntakeMidToIn().start();
      }
    } else if(targetState == CargoIntakeState.OUT) {
      if(currentState == CargoIntakeState.IN) {
        new CargoIntakeOutToIn().start();
      } else if(currentState == CargoIntakeState.MID) {
        new CargoIntakeMidToOut().start();
      }
    } else if(targetState == CargoIntakeState.MID) {
      if(currentState == CargoIntakeState.IN) {
        new CargoIntakeInToMid().start();
      } else if(currentState == CargoIntakeState.OUT) {
        new CargoIntakeOutToMid().start();
      }
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
