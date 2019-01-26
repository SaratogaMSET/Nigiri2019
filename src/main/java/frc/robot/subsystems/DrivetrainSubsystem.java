/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DrivetrainSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX[] motors;
  Encoder rightEncoder;
  Encoder leftEncoder;
  public double wheelSize = 4;
  public double scalingFactor = 4;

  public double kP, kI, kD, kF;

  public DrivetrainSubsystem() {
    motors = new TalonSRX[6];
    
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
    }

    rightEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[0], RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[1]);
    leftEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[0], RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[1]);
    
    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kF = 0.0;

    

    motors[1].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[2].set(ControlMode.Follower, motors[0].getDeviceID());  

    motors[4].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[5].set(ControlMode.Follower, motors[0].getDeviceID());
  }

  public void changeBrakeCoast(boolean isBrake) {
    if (isBrake) {
      motors[0].setNeutralMode(NeutralMode.Brake);
      motors[1].setNeutralMode(NeutralMode.Brake);
      motors[2].setNeutralMode(NeutralMode.Brake);
      motors[3].setNeutralMode(NeutralMode.Brake);
      motors[4].setNeutralMode(NeutralMode.Brake);
      motors[5].setNeutralMode(NeutralMode.Brake);
    }
    else {
      motors[0].setNeutralMode(NeutralMode.Coast);
      motors[1].setNeutralMode(NeutralMode.Coast);
      motors[2].setNeutralMode(NeutralMode.Coast);
      motors[3].setNeutralMode(NeutralMode.Coast);
      motors[4].setNeutralMode(NeutralMode.Coast);
      motors[5].setNeutralMode(NeutralMode.Coast);
    }
  }

  public void rawDrive(double left, double right) {
    motors[0].set(ControlMode.PercentOutput, right);
    motors[3].set(ControlMode.PercentOutput, left);
  }
  
  public int convertDistanceToTicks(double d) {
    int i = (int)((scalingFactor*d*50*4096)/(Math.PI*wheelSize*24));
    return i;
  }

  public int getLeftEncoder() {
    return leftEncoder.get();
  }

  public int getRightEncoder() {
    return rightEncoder.get();
  }

  public void resetEncoders() {
    motors[0].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
    motors[2].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
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