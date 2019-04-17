/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TunePIDFValues extends Command {

  Preferences prefs;
  public TunePIDFValues() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    prefs = Preferences.getInstance();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double kp = prefs.getDouble("kp", 0.5);
    double ki = prefs.getDouble("ki", 0.006);
    double kd = prefs.getDouble("kd", 5);
    double kf = prefs.getDouble("ki", 0.22);

    SmartDashboard.putNumber("Set kp", kp);
    SmartDashboard.putNumber("Set ki", ki);
    SmartDashboard.putNumber("Set kd", kd);
    SmartDashboard.putNumber("Set kf", kf);

    Robot.drive.motors[0].config_kP(0, kp, 20);
    Robot.drive.motors[0].config_kI(0, ki, 20);
    Robot.drive.motors[0].config_kD(0, kd, 20);
    Robot.drive.motors[0].config_kF(0, kf, 20);

    Robot.drive.motors[3].config_kP(0, kp, 20);
    Robot.drive.motors[3].config_kI(0, ki, 20);
    Robot.drive.motors[3].config_kD(0, kd, 20);
    Robot.drive.motors[3].config_kF(0, kf, 20);
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
