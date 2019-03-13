/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.util.RobotState;

public class LiftWhenSafe extends Command {
  LiftPositions target;
  public LiftWhenSafe(LiftPositions target) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.target = target;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    new ChangeIntakeState(CargoIntakePositionState.MID).start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return RobotState.cargoIntakeState == CargoIntakePositionState.MID;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(RobotState.cargoIntakeState == CargoIntakePositionState.MID) {
      new MoveLiftCommand(target, 1.2).start();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
