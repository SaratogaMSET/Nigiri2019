/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoDeploySubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX rightWheel;
  private TalonSRX leftWheel;
  
  public CargoDeploySubsystem() {
    rightWheel = new TalonSRX(RobotMap.CargoDeploy.rightMotor);
    leftWheel = new TalonSRX(RobotMap.CargoDeploy.leftMotor);
  }

  public void runMotors(double power) {
    rightWheel.set(ControlMode.PercentOutput, power);
    leftWheel.set(ControlMode.PercentOutput, -power);
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void diagnosticShuffleboard() {
    
  }

  @Override
  public void essentialShuffleboard() {
    
  }
}
