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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DrivetrainSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX[] motors;

  public double wheelSize = 4;
  public double scalingFactor = 4;

  public double kP, kI, kD, kF;

  public DrivetrainSubsystem() {
    motors = new TalonSRX[4];
    
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
    }

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kF = 0.0;

    motors[0].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Robot.timeoutMs);
    motors[0].configNominalOutputForward(0, Robot.timeoutMs);
    motors[0].configNominalOutputReverse(0, Robot.timeoutMs);
    motors[0].configPeakOutputForward(1, Robot.timeoutMs);
    motors[0].configPeakOutputReverse(-1, Robot.timeoutMs);
		motors[0].configMotionCruiseVelocity(20000, Robot.timeoutMs);
    motors[0].configMotionAcceleration(11000, Robot.timeoutMs);
    motors[0].config_kP(0, kP, Robot.timeoutMs);
    motors[0].config_kI(0, kI, Robot.timeoutMs);
    motors[0].config_kP(0, kP, Robot.timeoutMs);
    motors[0].config_kF(0, kF, Robot.timeoutMs);
    motors[0].setInverted(false);

    motors[1].configNominalOutputForward(0, Robot.timeoutMs);
    motors[1].configNominalOutputReverse(0, Robot.timeoutMs);
    motors[1].configPeakOutputForward(1, Robot.timeoutMs);
    motors[1].configPeakOutputReverse(-1, Robot.timeoutMs);
    motors[1].setInverted(false);
    motors[1].set(ControlMode.Follower, RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[0]);
    
		motors[2].configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Robot.timeoutMs);
    motors[2].configNominalOutputForward(0, Robot.timeoutMs);
    motors[2].configNominalOutputReverse(0, Robot.timeoutMs);
    motors[2].configPeakOutputForward(1, Robot.timeoutMs);
    motors[2].configPeakOutputReverse(-1, Robot.timeoutMs);
		motors[2].configMotionCruiseVelocity(20000, Robot.timeoutMs);
		motors[2].configMotionAcceleration(11000, Robot.timeoutMs);
    motors[2].config_kP(0, kP, Robot.timeoutMs);
    motors[2].config_kI(0, kI, Robot.timeoutMs);
    motors[2].config_kP(0, kP, Robot.timeoutMs);
    motors[2].config_kF(0, kF, Robot.timeoutMs);
    motors[2].setInverted(true);

    motors[3].configNominalOutputForward(0, Robot.timeoutMs);
    motors[3].configNominalOutputReverse(0, Robot.timeoutMs);
    motors[3].configPeakOutputForward(1, Robot.timeoutMs);
    motors[3].configPeakOutputReverse(-1, Robot.timeoutMs);
    motors[3].setInverted(true);
    motors[3].set(ControlMode.Follower, RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[2]);

  }

  public void changeBrakeCoast(boolean isBrake) {
    if (isBrake) {
      motors[0].setNeutralMode(NeutralMode.Brake);
      motors[1].setNeutralMode(NeutralMode.Brake);
      motors[2].setNeutralMode(NeutralMode.Brake);
      motors[3].setNeutralMode(NeutralMode.Brake);
    }
    else {
      motors[0].setNeutralMode(NeutralMode.Coast);
      motors[1].setNeutralMode(NeutralMode.Coast);
      motors[2].setNeutralMode(NeutralMode.Coast);
      motors[3].setNeutralMode(NeutralMode.Coast);
    }
  }

  public void rawDrive(double left, double right) {
    motors[0].set(ControlMode.PercentOutput, left);
    motors[2].set(ControlMode.PercentOutput, right);
  }
  
  public int convertDistanceToTicks(double d) {
    int i = (int)((scalingFactor*d*50*4096)/(Math.PI*wheelSize*24));
    return i;
  }

  public int getLeftEncoder() {
    return motors[0].getSelectedSensorPosition();
  }

  public int getRightEncoder() {
    return motors[2].getSelectedSensorPosition();
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
}