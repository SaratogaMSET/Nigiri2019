/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;

public class TestSensorsCommand extends Command {

  AnalogInput ultrasonic;
  DigitalInput button;
  I2C colorSensor;

  public TestSensorsCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    ultrasonic = new AnalogInput(0);
    button = new DigitalInput(5);
    colorSensor = new I2C(I2C.Port.kOnboard, 0x39);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("ultrasonic", ultrasonic.getVoltage());
    SmartDashboard.putBoolean("button", button.get());
    //colorSensor.read(0x39, 24, new ByteBuffer());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
