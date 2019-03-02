/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;

public class MotionProfileCommand extends FishyCommand {
  
  String pathName;

  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;
  
  private Notifier followerNotifier;
  
  private Trajectory leftTraj, rightTraj;
  private boolean isReversePath = false;


  public MotionProfileCommand(String pathName, boolean revPath) {
    super(Robot.drive, Robot.gyro); // call super with the subsystems that the command needs - this will block all other commands from using these subsystems while running

    this.pathName = pathName;
    this.isReversePath = revPath;
  }

  // The command MUST implement this method - the fields which you want to log
  protected String[] getLogFields() {
    // velocities for MP
    return new String[] {"Right Target", "Left Target", "Left Actual", "Right Actual", "Left Power", "Right Power"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.resetEncoders();
    Robot.gyro.resetGyro();
    runPath(pathName, this.isReversePath);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isMPFinish();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    logger.drain();
    logger.flush();
    if(followerNotifier != null) {
      followerNotifier.stop();
    }
    Robot.drive.rawDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public void runPath(String pathName, boolean reversePath) {
    Robot.drive.changeBrakeCoast(false);

    //TODO: When WPI changes it, change the left and right trajectories. the current version of Wpilib-Pathfinder has a bug
    
    rightTraj = PathfinderFRC.getTrajectory(pathName + ".left");
    leftTraj = PathfinderFRC.getTrajectory(pathName + ".right");

    leftFollower = new EncoderFollower(leftTraj);
    rightFollower = new EncoderFollower(rightTraj);

    if(reversePath) {
      leftFollower.configureEncoder(-Robot.drive.getRawLeftEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
      rightFollower.configureEncoder(-Robot.drive.getRawRightEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
    }
    else {
      leftFollower.configureEncoder(Robot.drive.getRawLeftEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
      rightFollower.configureEncoder(Robot.drive.getRawRightEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
    }

    leftFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Left.kp, 
                                0.0, 
                                DrivetrainSubsystem.PathFollowingConstants.Left.kd,
                                DrivetrainSubsystem.PathFollowingConstants.Left.kv,
                                DrivetrainSubsystem.PathFollowingConstants.Left.ka);

    rightFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Right.kp, 
                                0.0, 
                                DrivetrainSubsystem.PathFollowingConstants.Right.kd,
                                DrivetrainSubsystem.PathFollowingConstants.Right.kv,
                                DrivetrainSubsystem.PathFollowingConstants.Right.ka);

    this.isReversePath = reversePath;

    followerNotifier = new Notifier(this::followPath);
    followerNotifier.startPeriodic(leftTraj.get(0).dt);
  }

  private void followPath(){
    if(leftFollower.isFinished() && rightFollower.isFinished()){
      Robot.drive.isPathRunning = false;
      followerNotifier.stop();
      followerNotifier = null;
    } else {
      log("Right Target", rightFollower.getSegment().velocity);
      log("Left Target", leftFollower.getSegment().velocity);

      double leftSpeed, rightSpeed;
      if(isReversePath) {
        leftSpeed = -rightFollower.calculate(-Robot.drive.getRawRightEncoder());
        rightSpeed = -leftFollower.calculate(-Robot.drive.getRawLeftEncoder());
      }
      else {
        leftSpeed = leftFollower.calculate(Robot.drive.getRawLeftEncoder());
        rightSpeed = rightFollower.calculate(Robot.drive.getRawRightEncoder());
        
      }
      
      double heading = Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle());
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());

      double headingDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      double turn = DrivetrainSubsystem.PathFollowingConstants.kp_gyro * headingDiff;
      
      log("Right Actual", Robot.drive.rightEncoder.getRate());
      log("Left Actual", Robot.drive.leftEncoder.getRate());


      double left = leftSpeed + turn;
      double right = rightSpeed - turn;
      
      double normalizer = Math.max(Math.max(left, right), 1.0);

      // left /= normalizer;
      // right /= normalizer;

      if(rightFollower.getSegment().acceleration > 0.0 && rightFollower.getSegment().velocity < 1.5) {
        left = Math.signum(left) * Math.max(Math.abs(left), 0.6);
        right = Math.signum(right) * Math.max(Math.abs(right), 0.6);

        Robot.drive.rawDrive(left, right);

      }
      else {
        Robot.drive.rawDrive(left, right);
      }

      log("Left Power", left);
      log("Right Power", right);

      logger.write();

    }
  }

  public boolean isMPFinish(){
    return leftFollower.isFinished() && rightFollower.isFinished();
  }
}
