/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.FileWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.FishyMath;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;


/**
 * Add your docs here.
 */
public class DrivetrainSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // motors[0] is the right master, motors [1,2] right followers
  // motors[3] is the left master, motors [4,5] left followers

  // NEED TO RETUNE EVERY TIME ROBOT CHANGES
  // These are the values that go into path code PIDVA
  public static final double ROBOT_TRUE_MAX_VELOCITY = 12.0; // ft/s

  // wouldn't want to tip, now would we?
  // these are the values that go into pathfinder
  public static final double ROBOT_TARGET_MAX_VELOCITY = 9.0; // ft/s
  public static final double ROBOT_TARGET_MAX_ACCELERATION = 16.0; // ft/s^2

  public TalonSRX[] motors;

  public boolean isPathRunning;
  public int motor;


  public static final double WHEEL_DIAMETER = (4.06/12.0); // feet
  public static final int TICKS_PER_REV = 4096;
  public static final double WHEELBASE_FEET = 2.1804; // feet
  public static final double EMPIRICAL_WHEELBASE_FEET = 2.1804; // feet

  public DrivetrainSubsystem() {
    motors = new TalonSRX[6];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
      motors[i].configNominalOutputForward(0.08, 200);
      motors[i].configNominalOutputReverse(-0.08, 200);
      motors[i].configPeakOutputForward(1, 200);
      motors[i].configPeakOutputReverse(-1, 200);

      motors[i].configPeakCurrentDuration(300);
      motors[i].configPeakCurrentLimit(65, 20);
      motors[i].configContinuousCurrentLimit(40, 20);
      motors[i].enableCurrentLimit(true);
    }

    motors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 200);
    motors[0].selectProfileSlot(0, 0);
    motors[0].config_kF(0, 0.24);
    motors[0].config_kP(0, 1.5);
    motors[0].config_kI(0, 0.00);
    motors[0].config_kD(0, 5.0);


    motors[3].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 200);
    motors[3].selectProfileSlot(0, 0);
    motors[3].config_kF(0, 0.24);
    motors[3].config_kP(0, 1.5);
    motors[3].config_kI(0, 0.00);
    motors[3].config_kD(0, 5.0);

    // follow right master
    motors[1].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[2].set(ControlMode.Follower, motors[0].getDeviceID());

    // follow left master
    motors[4].set(ControlMode.Follower, motors[3].getDeviceID());
    motors[5].set(ControlMode.Follower, motors[3].getDeviceID());

    // invert right side DT motors
    motors[0].setInverted(true);
    motors[3].setInverted(false);

    motors[1].setInverted(InvertType.FollowMaster);
    motors[2].setInverted(InvertType.FollowMaster);
    motors[4].setInverted(InvertType.FollowMaster);
    motors[5].setInverted(InvertType.FollowMaster);
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
    motors[1].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[2].set(ControlMode.Follower, motors[0].getDeviceID());
    motors[4].set(ControlMode.Follower, motors[3].getDeviceID());
    motors[5].set(ControlMode.Follower, motors[3].getDeviceID());

    motors[0].set(ControlMode.PercentOutput, right);
    motors[3].set(ControlMode.PercentOutput, left);
  }

  public int getRawLeftEncoder() {
    return motors[3].getSelectedSensorPosition();
  }

  public void driveFwdRotate(double fwd, double rot){
		double left = fwd + rot, right = fwd - rot;
		double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
		left /= max;
		right /= max;

		rawDrive(left, right);
  }

  public int getRawRightEncoder() {
    return motors[0].getSelectedSensorPosition();
  }

  public synchronized double getRightEncoderVelocity() {
    return FishyMath.rpm2fps(FishyMath.talaonunits2rpm(motors[0].getSelectedSensorVelocity()));
  }

  public synchronized double getLeftEncoderVelocity() {
    return FishyMath.rpm2fps(FishyMath.talaonunits2rpm(motors[3].getSelectedSensorVelocity()));
  }

  public synchronized double getRightEncoderDistance() {
    return (motors[0].getSelectedSensorPosition() / (double) TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI;
  }

  public synchronized double getLeftEncoderDistance() {
    return (motors[3].getSelectedSensorPosition() / (double) TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI;
  }

  public void resetEncoders() {
    motors[0].setSelectedSensorPosition(0);
    motors[3].setSelectedSensorPosition(0);
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
      // SmartDashboard.putNumber("Motor", motors[motor].getDeviceID());
      // testMotor(fwd);
    }
    else {
      // SmartDashboard.putString("Motor", "all");
      testAllMotors(fwd);
    }
    // SmartDashboard.putNumber("Left Encoder", getRawLeftEncoder());
    // SmartDashboard.putNumber("Right Encoder", getRawRightEncoder());
  }

  public void testMotor(int num, double fwd) {
    motors[num].set(ControlMode.PercentOutput, fwd);
  }

  public void testAllMotors(double fwd) {
    motors[0].set(ControlMode.PercentOutput, fwd);
    motors[1].set(ControlMode.PercentOutput, fwd);
    motors[2].set(ControlMode.PercentOutput, fwd);
    motors[3].set(ControlMode.PercentOutput, fwd);
    motors[4].set(ControlMode.PercentOutput, fwd);
    motors[5].set(ControlMode.PercentOutput, fwd);
  }

  public void resetControlMode() {
    for (int i = 0; i < motors.length; i++) {
      motors[i].set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void diagnosticShuffleboard() {
    ShuffleboardTab drive = Shuffleboard.getTab("Drive");
    // drive.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kTextView);
    // drive.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kTextView);
  }

  @Override
  public void essentialShuffleboard() {
    ShuffleboardTab drive = Shuffleboard.getTab("Drive");
    // drive.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kTextView);
    // drive.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kTextView);
  }

  @Override
  public void stopAll() {
    rawDrive(0.0, 0.0);
  }
}
