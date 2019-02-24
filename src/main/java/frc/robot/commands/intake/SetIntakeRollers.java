/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetIntakeRollers extends Command {
  
  double power;
  boolean intake;
  public SetIntakeRollers(boolean intake, double power) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.power = power;
    this.intake = intake;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(intake) {
      Robot.cargoDeploy.runIntake(-power);
    } else {
      Robot.cargoDeploy.runIntake(power);
    }
    Robot.cargoIntake.runIntake(intake, power);

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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
