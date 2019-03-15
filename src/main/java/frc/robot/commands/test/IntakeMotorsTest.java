/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;

public class IntakeMotorsTest extends Command {

  Timer time;

  public IntakeMotorsTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time = new Timer();
    time.reset();
    time.start();
    SmartDashboard.putBoolean("Running Intake Test", true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.oi.gamePad.getPOVLeft()) { //left intake
      Robot.cargoIntake.runMotor(1, Robot.oi.gamePad.getRightJoystickY());
      SmartDashboard.putString("Currently Running Motor", "Motor 8 Left");
    } 
    else if(Robot.oi.gamePad.getPOVRight()) { //right intake
      Robot.cargoIntake.runMotor(2, Robot.oi.gamePad.getRightJoystickY());
      SmartDashboard.putString("Currently Running Motor", "Motor 9 Right");
    } 
    else if(Robot.oi.gamePad.getPOVUp()) { //back intake
      Robot.cargoIntake.runMotor(3, Robot.oi.gamePad.getRightJoystickY());
      SmartDashboard.putString("Currently Running Motor", "Motor 22 Back");
    }  else if(Robot.oi.gamePad.getButtonXPressed()) { //pistons in/out
      Robot.cargoIntake.setMidStateSol(true);
      SmartDashboard.putBoolean("Mid State", true);
    } else if(Robot.oi.gamePad.getButtonBPressed()) { //pistons in/out
      Robot.cargoIntake.setMidStateSol(false);
      SmartDashboard.putBoolean("Mid State", false);      
    } else if(Robot.oi.gamePad.getLeftButton()) { //pistons in/out
      Robot.cargoIntake.runIntake(true, Robot.oi.gamePad.getRightJoystickY());
      SmartDashboard.putString("Currently Running Motor", "All Intake Motors");
    } else if(Robot.oi.gamePad.getButtonYPressed()) {
      Robot.cargoIntake.switchSol(true);
    } else if(Robot.oi.gamePad.getButtonAPressed()) {
      Robot.cargoIntake.switchSol(false);
    } else if(Robot.oi.gamePad.getRightButtonPressed()) {
      new ChangeIntakeState(CargoIntakePositionState.IN).start();
    } else if(Robot.oi.gamePad.getRightTrigger()) {
      new ChangeIntakeState(CargoIntakePositionState.MID).start();
    } else if(Robot.oi.gamePad.getLeftTrigger()) {
      new ChangeIntakeState(CargoIntakePositionState.OUT).start();
    }
    else{
      Robot.cargoIntake.runMotor(1, 0);
      Robot.cargoIntake.runMotor(2, 0);
      Robot.cargoIntake.runMotor(3, 0);
    }
    SmartDashboard.putBoolean("out hal", Robot.cargoIntake.getOutHal());
    SmartDashboard.putBoolean("in hal", Robot.cargoIntake.getInHal());
    SmartDashboard.putBoolean("Mid State Sol", Robot.cargoIntake.getMidStateSolState());
    SmartDashboard.putBoolean("Pos State Sol", Robot.cargoIntake.getIntakeSolState());
    SmartDashboard.putString("Currently Running Diagnostic", "Cargo Intake");

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override

  protected boolean isFinished() {
    return Robot.oi.gamePad.getBackButton();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putBoolean("Running Intake Test", false);
    Robot.cargoIntake.runIntake(true, 0);
    new ChangeIntakeState(CargoIntakePositionState.MID).start();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
