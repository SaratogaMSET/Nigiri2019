/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.Robot;

/**
 * Add your docs here.
 */


public class JackSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private TalonSRX jackMotor;

  public JackSubsystem(){
    jackMotor = new TalonSRX(RobotMap.Jacks);
    jackMotor.setInverted(false);
    jackMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Robot.timeoutMs);
  }

  public void setJackMotor(double pow){
    jackMotor.set(ControlMode.PercentOutput, pow);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
