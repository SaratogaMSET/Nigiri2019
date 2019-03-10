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
  public static class DriveStraightGyroConstants {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double cumError = 0;
    public static double lastError = 0;
  }

  public static class DriveStraightConstants {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double cumError = 0;
    public static double lastError = 0;
  }

  public static class PathFollowingConstants {
    public static class Forward {
      public static class Left { // tuned
        public static final double kp = 1.8;
        public static final double kd = 0.2;
        public static final double kv = 0.11; //tuned
        public static final double ka = 0.055;
      }
      public static class Right { // tuned
        public static final double kp = 1.8;
        public static final double kd = 0.2;
        public static final double kv = 0.11; // tuned
        public static final double ka = 0.055;
      }
    }
    public static class Reverse {
      public static class Left { // tuned
        public static final double kp = 1.2;
        public static final double kd = 0.1;
        public static final double kv = 0.11; //tuned
        public static final double ka = 0.055;
      }
      public static class Right { // tuned
        public static final double kp = 1.6;
        public static final double kd = 0.0;
        public static final double kv = 0.11; // tuned
        public static final double ka = 0.055;
      }
    }
    public static final double kp_gyro = 0.015;
  }


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

  public double kP, kI, kD, kF;


  public DrivetrainSubsystem() {
    motors = new TalonSRX[6];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
      motors[i].configNominalOutputForward(0.02, Robot.timeoutMs);
      motors[i].configNominalOutputReverse(-0.02, Robot.timeoutMs);
      motors[i].configPeakOutputForward(1, Robot.timeoutMs);
      motors[i].configPeakOutputReverse(-1, Robot.timeoutMs);

    }
    /*
    rightEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[0], RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[1], false, EncodingType.k1X);
    leftEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[0], RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[1], true, EncodingType.k1X);
    
    rightEncoder.setDistancePerPulse(WHEEL_DIAMETER * Math.PI / (double) TICKS_PER_REV);
    leftEncoder.setDistancePerPulse(WHEEL_DIAMETER * Math.PI /  (double) TICKS_PER_REV);

    leftEncoder.setSamplesToAverage(127);
    rightEncoder.setSamplesToAverage(127);

    leftEncoder.setMinRate((0.2/12.0));
    rightEncoder.setMinRate((0.2/12.0));

    */

    motors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motors[3].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    kP = 0.0;
    kI = 0.0;
    kD = 0.0;
    kF = 0.0;

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


    motor = 0;
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

  public double getRightEncoderVelocity() {
    return (double) ((double) Robot.drive.motors[0].getSelectedSensorVelocity() * (10.0 / (double) DrivetrainSubsystem.TICKS_PER_REV)) * Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER;
  }

  public double getLeftEncoderVelocity() {
    return (double) ((double) Robot.drive.motors[3].getSelectedSensorVelocity() * (10.0 / (double) DrivetrainSubsystem.TICKS_PER_REV)) * Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER;
  }

  public double getRightEncoderDistance() {
    return (motors[0].getSelectedSensorPosition() / (double) TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI;
  }

  public double getLeftEncoderDistance() {
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
      SmartDashboard.putNumber("Motor", motors[motor].getDeviceID());
      testMotor(fwd);
    }
    else {
      SmartDashboard.putString("Motor", "all");
      testAllMotors(fwd);
    }
    SmartDashboard.putNumber("Left Encoder", getRawLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", getRawRightEncoder());
  }

  public void testMotor(double fwd) {
    motors[motor].set(ControlMode.PercentOutput, fwd);
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

  
  
  public double getGyroStraightPIDOutput(double error) {
    double p = DriveStraightGyroConstants.kp * error;
    DriveStraightGyroConstants.cumError += error;
    double i = DriveStraightGyroConstants.ki * DriveStraightGyroConstants.cumError;
    double dError = error - DriveStraightGyroConstants.lastError;
    DriveStraightGyroConstants.lastError = error;
    double d = DriveStraightGyroConstants.kd * dError;

    return p + i + d;
  }

  public double getStraightPIDOutput(double error) {
    double p = DriveStraightConstants.kp * error;
    DriveStraightConstants.cumError += error;
    double i = DriveStraightConstants.ki * DriveStraightConstants.cumError;
    double dError = error - DriveStraightConstants.lastError;
    DriveStraightConstants.lastError = error;
    double d = DriveStraightConstants.kd * dError;

    return p + i + d;
  }

  public void PIDReset() {
    DriveStraightConstants.cumError = 0;
    DriveStraightGyroConstants.cumError = 0;
    DriveStraightConstants.lastError = 0;
    DriveStraightGyroConstants.lastError = 0;
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