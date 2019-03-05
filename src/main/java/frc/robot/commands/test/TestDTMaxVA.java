/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.FishyCommand;

public class TestDTMaxVA extends FishyCommand {
  double timeout;
  Timer time;

  double prevTime = 0.0;
  double prevRV = 0.0;
  double prevLV = 0.0;

  @Override
  protected String[] getLogFields() {
      return new String[] {"Right Vel", "Left Vel", "Right A", "Left A", "NavX Accel"};
  }

  public TestDTMaxVA(double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.timeout = timeout;
    time = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
    Robot.drive.changeBrakeCoast(true);
    Robot.drive.rawDrive(1.0, 1.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // log("Right Vel", Robot.drive.rightEncoder.getRate());
    // log("Left Vel", Robot.drive.leftEncoder.getRate());
    // log("Right A", (Robot.drive.rightEncoder.getRate() - prevRV)/(time.get() - prevTime));
    // log("Left A", (Robot.drive.leftEncoder.getRate() - prevLV)/(time.get() - prevTime));

    // prevTime = time.get();
    // prevLV = Robot.drive.leftEncoder.getRate();
    // prevRV = Robot.drive.rightEncoder.getRate();

    log("NavX Accel", Robot.gyro.gyro.getWorldLinearAccelY() * 32.17405); // ft/s^2

    logger.write();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (time.get() >= timeout) {
      return true;
    }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.rawDrive(0.0, 0.0);
    logger.drain();
    logger.flush();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      end();
  }
}
