/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
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

  //TESTING PRIDEBOT
  TalonSRX leftDriveA;
	TalonSRX leftDriveB;
	TalonSRX rightDriveA;
  TalonSRX rightDriveB;
  //
	
	public DrivetrainSubsystem() {
    //TESTING PRIDEBOT
		leftDriveA = new TalonSRX(13);
		leftDriveA.setInverted(true);
		leftDriveB = new TalonSRX(14);
		//leftDriveB.setInverted(true);
		rightDriveA = new TalonSRX(15);
		rightDriveB = new TalonSRX(16);
	  rightDriveB.set(ControlMode.Follower, 15);
    leftDriveB.set(ControlMode.Follower, 13);
    //

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

    motors[3].setInverted(InvertType.InvertMotorOutput);
    motors[4].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[4].setInverted(InvertType.InvertMotorOutput);
    motors[5].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[5].setInverted(InvertType.InvertMotorOutput);
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

  //TESTING PRIDEBOT
  public void arcadeDrive(double power, double rotation) {
		double leftPower = power-rotation;
		double rightPower = power+rotation;
		double maxe = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
		leftPower/=maxe;
		rightPower/=maxe;
		leftDriveA.set(ControlMode.PercentOutput, leftPower);
	  rightDriveA.set(ControlMode.PercentOutput, rightPower);
  }
  //
  
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
    rightEncoder.reset();
    leftEncoder.reset();
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