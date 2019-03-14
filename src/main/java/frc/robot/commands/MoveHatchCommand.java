/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.util.RobotState;
import frc.robot.util.Logging;

public class MoveHatchCommand extends Command {

  HatchPositionState hatchPosition;
  Timer timer;
  double waitTime;

  public MoveHatchCommand(HatchPositionState hatchPosition) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.hatchPosition = hatchPosition;
    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if((RobotState.cargoIntakeState != CargoIntakePositionState.MID) && hatchPosition == HatchPositionState.HATCH_OUT) {
      waitTime = 1;
    } else { 
      waitTime = 0;
    }


    if(Robot.isLogging) {
      String string = String.format("%.4f, MoveHatchCommand," + hatchPosition.toString(), Robot.time.get());
      Logging.print(string);
    }
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() >= waitTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(RobotState.cargoIntakeState == CargoIntakePositionState.MID) {
      Robot.hatch.moveHatch(hatchPosition);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
