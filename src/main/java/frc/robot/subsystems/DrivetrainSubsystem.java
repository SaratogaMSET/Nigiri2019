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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SpeedController;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * Add your docs here.
 */
public class DrivetrainSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // motors[0] is the right master
  // motors[3] is the left master
  public TalonSRX[] motors;
  public Encoder rightEncoder;
  public Encoder leftEncoder;
  public boolean isPathRunning;


  private static final double wheelSize = 4;
  private static final double scalingFactor = 4;
  private static final int tickPerRev = 1024;

  public double kP, kI, kD, kF;


  private SpeedController leftMotorSpeedController;
  private SpeedController rightMotorSpeedController;

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
    leftEncoder = new Encoder(RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[0], RobotMap.Drivetrain.DRIVE_LEFT_ENCODER[1]);
    
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
    motors[3].setSelectedSensorPosition(0, 0, Robot.timeoutMs);
  }

  public void setTrajectory(String pathName, double p, double i, double d, double v, turnVal){
    leftTraj = PathfinderFRC.getTrajectory(pathName + ".left");
    rightTraj = PathfinderFRC.getTrajectory(pathName + ".right");

    leftFollower = new EncoderFollower(leftTraj);
    rightFollower = new EncoderFollower(rightTraj);

    leftFollower.configureEncoder(leftEncoder.get(), tickPerRev, wheelSize);
    rightFollower.configureEncoder(rightEncoder.get(), tickPerRev, wheelSize);

    leftFollower.configurePIDVA(p, i, d, v, 0);
    rightFollower.configurePIDVA(p, i, d, v, 0);

    followerNotifier = new Notifier(this::followPath);
    followerNotifier.startPeriodic(leftTraj.get(0).dt);

    turnMult = turnVal;
  }

  private void followPath(){
    if(leftFollower.isFinished() || rightFollower.isFinished()){
      isPathRunning = false;
      followerNotifier.stop();
    }else{
      double leftSpeed = leftFollower.calculate(leftEncoder.get());
      double rightSpeed = rightFollower.calculate(rightEncoder.get());

      double heading = 0; //add when we put in gyro
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());

      double headingDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      double turn = turnMult * (-1.0/80) * headingDiff;
      rawDrive(leftSpeed + turn, rightSpeed = turn);
    }
  }
  public void stopMP(){
    followerNotifier.stop();
    rawDrive(0,0);
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