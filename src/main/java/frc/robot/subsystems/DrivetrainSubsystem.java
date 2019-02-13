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

  // NEED TO RETUNE EVERY TIME ROBOT CHANGES
  // These are the values that go into path code PIDVA
  public static final double ROBOT_TRUE_MAX_VELOCITY = 12.0; // ft/s
  public static final double ROBOT_TRUE_MAX_ACCELERATION = 22.0; // ft/s^2

  // wouldn't want to tip, now would we?
  // these are the values that go into pathfinder
  public static final double ROBOT_TARGET_MAX_VELOCITY = 12.0; // ft/s
  public static final double ROBOT_TARGET_MAX_ACCELERATION = 15.0; // ft/s^2

  public TalonSRX[] motors;
  public Encoder rightEncoder;
  public Encoder leftEncoder;
  public boolean isPathRunning;
  public int motor;


  private static final double wheelDiameter = (4.1/12.0); // feet
  private static final int tickPerRev = 1024;

  public double kP, kI, kD, kF;

  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;
  
  private Notifier followerNotifier;
  
  private Trajectory leftTraj, rightTraj;
  
  private double turnMult;
  private boolean isReversePath = false;

  // Path logging
  private double[] targetVel = new double[10000];
  private double[] actualVel = new double[10000];

  public DrivetrainSubsystem() {
    motors = new TalonSRX[6];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
    }
    rightEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[0], RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[1]);
    leftEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[1], RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[0]);
    
    rightEncoder.setDistancePerPulse(wheelDiameter * Math.PI / (double) tickPerRev);
    leftEncoder.setDistancePerPulse(wheelDiameter * Math.PI /  (double) tickPerRev);
    
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
    motors[0].setInverted(InvertType.InvertMotorOutput);
    motors[1].setInverted(InvertType.InvertMotorOutput);
    motors[2].setInverted(InvertType.InvertMotorOutput);

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
    motors[0].set(ControlMode.PercentOutput, right);
    motors[3].set(ControlMode.PercentOutput, left);
  }
  
  public int getRawLeftEncoder() {
    return leftEncoder.get();
  }

  public void driveFwdRotate(double fwd, double rot){
		double left = fwd + rot, right = fwd - rot;
		double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
		left /= max;
		right /= max;
		
		rawDrive(left, right);
  }
  
  public int getRawRightEncoder() {
    return rightEncoder.get();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void runPath(String pathName, double p, double i, double d, double v, double a, double turnVal, boolean revPath) {
    changeBrakeCoast(false);
    //TODO: When WPI changes it, change the left and right trajectories. the current version of Wpilib-Pathfinder has a bug
    rightTraj = PathfinderFRC.getTrajectory(pathName + ".left");
    leftTraj = PathfinderFRC.getTrajectory(pathName + ".right");

    leftFollower = new EncoderFollower(leftTraj);
    rightFollower = new EncoderFollower(rightTraj);

    if(isReversePath) {
      leftFollower.configureEncoder(getRawLeftEncoder(), tickPerRev, wheelDiameter);
      rightFollower.configureEncoder(getRawRightEncoder(), tickPerRev, wheelDiameter);
    }
    else {
      leftFollower.configureEncoder(getRawLeftEncoder(), tickPerRev, wheelDiameter);
      rightFollower.configureEncoder(getRawRightEncoder(), tickPerRev, wheelDiameter);
    }


    double kv = (v == 0.0) ? 0.0 : 1/v;
    double ka = (a == 0.0) ? 0.0 : a;
    leftFollower.configurePIDVA(p, i, d, kv, ka);
    rightFollower.configurePIDVA(p, i, d, kv, ka);

    turnMult = turnVal;
    isReversePath = revPath;

    followerNotifier = new Notifier(this::followPath);
    followerNotifier.startPeriodic(leftTraj.get(0).dt);
    segmentCount = 0;
    velCount = 0;
  }

  private int segmentCount = 0;

  private void followPath(){
    if(leftFollower.isFinished() && rightFollower.isFinished()){
      SmartDashboard.putNumber("FINISHED", 1);
      isPathRunning = false;
      followerNotifier.stop();
      rawDrive(0, 0);
      followerNotifier = null;
      logPathVel();
    }else{
      SmartDashboard.putNumber("FINISHED",0);
      double leftSpeed, rightSpeed;
      if(isReversePath) {
        leftSpeed = -rightFollower.calculate(-rightEncoder.get());
        rightSpeed = -leftFollower.calculate(-leftEncoder.get());
      }
      else {
        leftSpeed = leftFollower.calculate(leftEncoder.get());
        rightSpeed = rightFollower.calculate(rightEncoder.get());
        
      }
      
      double heading = Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle());
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());

      double headingDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      SmartDashboard.putNumber("HEADING DIFF", headingDiff); // TODO remove -  DEBUG
      double turn = turnMult * headingDiff;
  
      SmartDashboard.putNumber("REG", leftSpeed); // TODO remove - debug
      SmartDashboard.putNumber("TURN", turn); // TODO remove - debug

      NetworkTableInstance.getDefault().getTable("Paths").getEntry("TargetVelocity").setNumber(leftTraj.segments[segmentCount].velocity);
      NetworkTableInstance.getDefault().getTable("Paths").getEntry("ActualVelocity").setNumber(leftEncoder.getRate());
      trackVelocity(rightEncoder.getRate(), rightTraj.segments[segmentCount].velocity);

      segmentCount++;

      rawDrive(leftSpeed + turn, rightSpeed - turn);
    }
  }

  private int velCount = 0;
  public void trackVelocity(double actual, double target) {
    targetVel[velCount] = target;
    actualVel[velCount] = actual;
    velCount++;
  }

  public void logPathVel() {
    try {
      BufferedWriter br = new BufferedWriter(new FileWriter("/home/lvuser/path_velocity.csv"));
      StringBuilder sb = new StringBuilder();
      sb.append("Target Velolcity");
      sb.append(", Actual Velocity");
      sb.append("\n");
      for(int i = 0; i < targetVel.length; i++) {
        sb.append(targetVel[i]);
        sb.append(",");
        sb.append(actualVel[i]);
        sb.append("\n");
      }
      br.write(sb.toString());
      br.flush();
      br.close();
      
    }
    catch(Exception e) {
      SmartDashboard.putString("FAILED LOG", e.getLocalizedMessage());
    }

  }


  public boolean isMPFinish(){
    return leftFollower.isFinished() && rightFollower.isFinished();
  }
  public void stopMP(){
    if(followerNotifier != null) {
      followerNotifier.stop();
    }
    rawDrive(0,0);
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
    drive.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kTextView);
    drive.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kTextView);
  }

  @Override
  public void essentialShuffleboard() {
    ShuffleboardTab drive = Shuffleboard.getTab("Drive");
    drive.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kTextView);
    drive.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kTextView);
  }
}