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
import frc.robot.util.RobotState;
import frc.robot.util.Logging;

public class SetIntakeRollers extends Command {
  boolean intake;

  double sidePower;
  double topPower;
  double carriagePower;

  public SetIntakeRollers(boolean intake, double power) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.topPower = power;
    this.sidePower = power;
    this.carriagePower = power;
    this.intake = intake;
  }

  public SetIntakeRollers(boolean intake, double topRollerPower, double sideRollersPower, double carriagePower) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.topPower = topRollerPower;
    this.sidePower = sideRollersPower;
    this.carriagePower = carriagePower;
    this.intake = intake;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(intake) {
      Robot.cargoDeploy.runIntake(carriagePower);
    } else {
      Robot.cargoDeploy.runIntake(-carriagePower);
    }
    Robot.cargoIntake.runIntake(intake, topPower, sidePower);
    String string = String.format("%.2f %.2f %.2f", topPower, sidePower, carriagePower);
    SmartDashboard.putString("Run Intakes", string);

    if(Robot.isLogging) {
      String log = String.format("%f, SetIntakeRollers," + string, Robot.time.get());
      Logging.print(log);
    }
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
