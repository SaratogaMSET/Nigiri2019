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
    if(Robot.oi.driver.getDriverButton2()) { //left intake
      Robot.cargoIntake.runMotor(1, Robot.oi.driver.getDriverVertical());
      SmartDashboard.putString("Currently Running Motor", "Motor 8 Left");
    } 
    else if(Robot.oi.driver.getDriverButton3()) { //right intake
      Robot.cargoIntake.runMotor(2, Robot.oi.driver.getDriverVertical());
      SmartDashboard.putString("Currently Running Motor", "Motor 9 Right");
    } 
    else if(Robot.oi.driver.getDriverButton4()) { //back intake
      Robot.cargoIntake.runMotor(3, Robot.oi.driver.getDriverVertical());
      SmartDashboard.putString("Currently Running Motor", "Motor 22 Back");
    } 
    else if(Robot.oi.driver.getDriverButton5() && time.get() > 0.5) { //pistons in/out
      // Robot.cargoIntake.switchSol();

      time.reset();
      
    } else if(Robot.oi.driver.getDriverButton7() && time.get() > 0.5) { //pistons in/out
      Robot.cargoIntake.setMidStateSol(true);
      SmartDashboard.putBoolean("Mid State", true);
      time.reset();
      
    } else if(Robot.oi.driver.getDriverButton8() && time.get() > 0.5) { //pistons in/out
      Robot.cargoIntake.setMidStateSol(false);
      SmartDashboard.putBoolean("Mid State", false);
      time.reset();
      
    } else if(Robot.oi.driver.getDriverButton6()) { //pistons in/out
      Robot.cargoIntake.testAll(Robot.oi.driver.getDriverVertical());
      SmartDashboard.putString("Currently Running Motor", "All Intake Motors");
      
    }
    else{
      Robot.cargoIntake.runMotor(1, 0);
      Robot.cargoIntake.runMotor(2, 0);
      Robot.cargoIntake.runMotor(3, 0);
    }
    SmartDashboard.putBoolean("out hal", Robot.cargoIntake.getOutHal());
    SmartDashboard.putBoolean("Mid State Sol", Robot.cargoIntake.intakeMidSol.get());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override

  protected boolean isFinished() {
    return Robot.oi.driver.getDriverButton1();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    time.stop();
    SmartDashboard.putBoolean("Running Intake Test", false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
