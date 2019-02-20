/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;
import frc.robot.util.RobotState;

public class SetCargoIntakes extends Command {

  CargoIntakeState targetState;
  public SetCargoIntakes(CargoIntakeState targetState) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.targetState = targetState;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    CargoIntakeState currentState = RobotState.intakeState;
    
    if(targetState == CargoIntakeState.IN) {
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
