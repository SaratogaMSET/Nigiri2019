/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.util.RobotState;

public class SetIntakePistons extends Command {

  boolean out;
  boolean isFinished;
  Timer time;
  final double timeout;
  public SetIntakePistons(boolean out) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.out = out;
    timeout=2;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time = new Timer();
    time.reset();
    time.start();

    isFinished = false;

    if(Robot.cargoIntake.getIntakeSolState() == out) {
      isFinished = true;
    } else {
      Robot.cargoIntake.switchSol(out);
    }
    
    // SmartDashboard.putBoolean("SetIntakePistons Done?", false);
    // SmartDashboard.putBoolean("Intake Timeout?", false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(time.get() > timeout) {
      isFinished = true;
      // SmartDashboard.putBoolean("Intake Timeout?", true);
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(out) {
      if(RobotState.cargoIntakeState == CargoIntakePositionState.OUT) {
        return true;
      }
    } else if(RobotState.cargoIntakeState == CargoIntakePositionState.MID || 
              RobotState.cargoIntakeState == CargoIntakePositionState.IN) {
      return true;
    }
    return isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // SmartDashboard.putBoolean("SetIntakePistons Done?", true);
    // SmartDashboard.putString("Current Intake", RobotState.cargoIntakeState.toString());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
