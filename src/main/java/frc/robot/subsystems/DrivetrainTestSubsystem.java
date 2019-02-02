/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DrivetrainTestSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  TalonSRX[] motors;
  int motor;
  Encoder rightEncoder;
  Encoder leftEncoder;

  public DrivetrainTestSubsystem() {
    motors = new TalonSRX[6];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
    }
    motors[0].setInverted(InvertType.InvertMotorOutput);
    motors[1].setInverted(InvertType.InvertMotorOutput);
    motors[2].setInverted(InvertType.InvertMotorOutput);
    rightEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[0], RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[1]);
    leftEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[0], RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[1]);
    motor = 0;
  }

  public void testDrivetrain(double fwd, boolean changeMotor, boolean resetEncoders) {
    if (resetEncoders) {
      resetEncoders();
    }
    if (changeMotor) {
      if (motor < 6) {
        motor++;
      }
      else {
        motor = 0;
      }
    }
    if (motor < 6) {
      SmartDashboard.putNumber("Motor", motor);
      testMotor(fwd);
    }
    else {
      SmartDashboard.putString("Motor", "all");
      testAllMotors(fwd);
    }
    SmartDashboard.putNumber("Left Encoder", getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", getRightEncoder());
  }

  public void testMotor(double fwd) {
    motors[motor].set(ControlMode.PercentOutput, fwd);
  }

  public void testAllMotors(double fwd) {
    motors[1].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[2].set(ControlMode.Follower, motors[1].getDeviceID());
    motors[4].set(ControlMode.Follower, motors[3].getDeviceID());
    motors[5].set(ControlMode.Follower, motors[4].getDeviceID());
  }

  public int getRightEncoder() {
    return rightEncoder.get();
  }

  public int getLeftEncoder() {
    return leftEncoder.get();
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
}
