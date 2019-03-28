/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.FishyMath;

public class GyroPIDCommand extends Command {

  Timer time;
  Timer timer;
  double timeout;

  double targetAngle;

  PIDController pidController;

  public GyroPIDCommand(double targetAngle, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.targetAngle = targetAngle;
    this.timeout = timeout;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time = new Timer();
    time.reset();
    time.start();

    timer = new Timer();

    pidController = new PIDController(0.05, 0.0, 0.6, new PIDSource() {
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {}
      @Override
      public PIDSourceType getPIDSourceType() { return PIDSourceType.kDisplacement; }
    
      @Override
      public double pidGet() {
        return FishyMath.boundThetaNeg180to180(Robot.gyro.getGyroAngle());
      }
    }, new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        Robot.gyro.gyroPIDOutput = output;
      }
    }, 0.01);

    pidController.setAbsoluteTolerance(1);
    pidController.setInputRange(-180.0f, 180.0f);
    pidController.setOutputRange(-1, 1);
    pidController.setContinuous(true);
    pidController.setSetpoint(targetAngle);
    pidController.enable();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Gyro PID Output", Robot.gyro.getGyroPIDOutput());
    Robot.drive.rawDrive(Robot.gyro.getGyroPIDOutput(), -Robot.gyro.getGyroPIDOutput());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (pidController.onTarget() && timer.get() > 0.01) {
      timer.start();
    } else if(!pidController.onTarget() && time.get()> 0.01) {
      timer.stop();
      timer.reset();
    }
    if(timer.get() > 0.2) {
      return true;
    }
    else if (time.get() > timeout) {
      Robot.drive.rawDrive(0, 0);
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pidController.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
