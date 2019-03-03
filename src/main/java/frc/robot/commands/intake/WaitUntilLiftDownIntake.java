/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.MoveHatchCommand;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;
import frc.robot.subsystems.HatchSubsystem.HatchState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.util.RobotState;

public class WaitUntilLiftDownIntake extends Command {

  boolean intake;

  double sidePower;
  double topPower;
  double carriagePower;
  double timeout;

  Timer timer;

  public WaitUntilLiftDownIntake(boolean intake, double topRollerPower, double sideRollersPower, double carriagePower, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.topPower = topRollerPower;
    this.sidePower = sideRollersPower;
    this.carriagePower = carriagePower;
    this.intake = intake;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return timer.get() > timeout || RobotState.liftPosition == LiftPositions.CARGO_LOW || RobotState.liftPosition == LiftPositions.HATCH_LOW;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    new MoveHatchCommand(HatchState.hatchIn).start();
    new SetIntakeRollers(intake, topPower, sidePower, carriagePower).start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
