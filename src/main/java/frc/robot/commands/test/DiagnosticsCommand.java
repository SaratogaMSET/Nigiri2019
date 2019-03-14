/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DiagnosticsCommand extends Command {
  Command [] tests = { new CargoDeployTest(), new DrivetrainTest(),
  new HatchTest(), new IntakeMotorsTest(), new LedTest(), new LiftTest(), new TestSensorsCommand()};
 
  public DiagnosticsCommand() {
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    for(Command c : tests){
      while(!Robot.oi.gamePad.getRightButton()){
        //wait for button press
      }
      c.start();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.oi.gamePad.getStartButton();
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
