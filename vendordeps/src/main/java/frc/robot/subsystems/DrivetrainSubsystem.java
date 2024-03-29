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
import jaci.pathfinder.followers.EncoderFollower;

/**
 * Add your docs here.
 */
public class DrivetrainSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // motors[0] is the right master, motors [1,2] right followers
  // motors[3] is the left master, motors [4,5] left followers
  public TalonSRX[] motors;
  public Encoder rightEncoder;
  public Encoder leftEncoder;
  public boolean isPathRunning;


  private static final double wheelDiameter = (4.1/12.0); // feet
  private static final int tickPerRev = 1024;

  public double kP, kI, kD, kF;

  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;
  
  private Notifier followerNotifier;
  
  private Trajectory leftTraj, rightTraj;
  
  private static double turnMult;


  public DrivetrainSubsystem() {
    motors = new TalonSRX[6];
    for (int i = 0; i < motors.length; i++) {
      motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
    }
    rightEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[0], RobotMap.Drivetrain.DRIVE_RIGHT_ENCODER[1]);
    leftEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[1], RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[0]);
    
    rightEncoder.setDistancePerPulse(0.0010226538585612); // ft
    leftEncoder.setDistancePerPulse(0.0010226538585612); // ft

    

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
    motors[3].set(ControlMode.PercentOutput, left);
    motors[0].set(ControlMode.PercentOutput, right);
  }

  public void driveFwdRotate(double fwd, double roti){
		double rot = roti * roti;
		if(roti < 0){
			rot = -1*rot;
		}
		double left = fwd + rot, right = fwd - rot;
		double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
		left /= max;
		right /= max;
		
		rawDrive(left, right);
	}

  public int getLeftEncoder() {
    return leftEncoder.get(); // the left encoder is reversed
  }

  public int getRightEncoder() {
    return rightEncoder.get();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void setTrajectory(String pathName, double p, double i, double d, double v, double turnVal){
    changeBrakeCoast(true);
    //TODO: When WPI changes it, change the left and right trajectories. the current version of Wpilib-Pathfinder has a bug
    rightTraj = PathfinderFRC.getTrajectory(pathName + ".left");
    leftTraj = PathfinderFRC.getTrajectory(pathName + ".right");

    leftFollower = new EncoderFollower(leftTraj);
    rightFollower = new EncoderFollower(rightTraj);

    leftFollower.configureEncoder(getLeftEncoder(), tickPerRev, wheelDiameter);
    rightFollower.configureEncoder(getRightEncoder(), tickPerRev, wheelDiameter);

    leftFollower.configurePIDVA(p, i, d, 1.0/v, 0.0);
    rightFollower.configurePIDVA(p, i, d, 1.0/v, 0.0);

    turnMult = turnVal;

    followerNotifier = new Notifier(this::followPath);
    followerNotifier.startPeriodic(leftTraj.get(0).dt);
  }

  private void followPath(){
    if(leftFollower.isFinished() && rightFollower.isFinished()){
      isPathRunning = false;
      followerNotifier.stop();
      rawDrive(0, 0);
    }else{
      double leftSpeed = leftFollower.calculate(getLeftEncoder());
      double rightSpeed = rightFollower.calculate(getRightEncoder());

      double heading = Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle()); //add when we put in gyro
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
      double headingDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      SmartDashboard.putNumber("HEADING DIFF", headingDiff); // TODO remove -  DEBUG
      double turn = turnMult * headingDiff;

      double normalizer = Math.max(1, Math.max(Math.abs(leftSpeed + turn), Math.abs(rightSpeed - turn)));
      SmartDashboard.putNumber("NORMALIZER", leftSpeed + turn); // TODO remove - debug
      double leftNormalized  = (leftSpeed + turn) / normalizer;
      double rightNormalized = (rightSpeed - turn) / normalizer;

      rawDrive(leftNormalized, rightNormalized);
    }
  }
  public void stopMP(){
    if(followerNotifier != null) {
      followerNotifier.stop();
    }
    rawDrive(0,0);
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