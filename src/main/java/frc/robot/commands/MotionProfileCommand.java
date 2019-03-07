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
import frc.robot.RobotMap.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

import jaci.pathfinder.*;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;

public class MotionProfileCommand extends FishyCommand {
  
  String pathName;

  private EncoderFollower leftFollower;
  private EncoderFollower rightFollower;
  
  private Notifier followerNotifier;
  
  private Trajectory leftTraj, rightTraj;
  private boolean isReversePath;
  private boolean robotStartedBackwards;


  public MotionProfileCommand(String pathName, boolean revPath, boolean robotStartedBackwards) {
    super(Robot.drive, Robot.gyro); // call super with the subsystems that the command needs - this will block all other commands from using these subsystems while running

    this.pathName = pathName;
    this.isReversePath = revPath;
    this.robotStartedBackwards = robotStartedBackwards;
  }

  // The command MUST implement this method - the fields which you want to log
  protected String[] getLogFields() {
    // velocities for MP
    return new String[] {"Right Target", "Left Target", "Left Actual", "Right Actual", "Left Power", "Right Power"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
    Robot.drive.rawDrive(0.0, 0.0);
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
    if(reversePath) {
      for(Segment s : rightTraj.segments) {
        s.acceleration = -s.acceleration;
        s.jerk = -s.jerk;
        s.velocity = -s.velocity;
        s.position = -s.position;
      }
      for(Segment s : leftTraj.segments) {
        s.acceleration = -s.acceleration;
        s.jerk = -s.jerk;
        s.velocity = -s.velocity;
        s.position = -s.position;
      }
    }
    

    leftFollower = new EncoderFollower(leftTraj);
    rightFollower = new EncoderFollower(rightTraj);

    if(reversePath) {
      leftFollower.configureEncoder(Robot.drive.getRawRightEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
      rightFollower.configureEncoder(Robot.drive.getRawLeftEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);

    }
    else {
      leftFollower.configureEncoder(Robot.drive.getRawLeftEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
      rightFollower.configureEncoder(Robot.drive.getRawRightEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
    }
    
    if(reversePath) {
      leftFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.kp, 
                                0.0, 
                                DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.kd,
                                DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.kv,
                                DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.ka);

      rightFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.kp, 
                                0.0, 
                                DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.kd,
                                DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.kv,
                                DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.ka);

    }
    else {
      leftFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Forward.Left.kp, 
                                0.0, 
                                DrivetrainSubsystem.PathFollowingConstants.Forward.Left.kd,
                                DrivetrainSubsystem.PathFollowingConstants.Forward.Left.kv,
                                DrivetrainSubsystem.PathFollowingConstants.Forward.Left.ka);

      rightFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Forward.Right.kp, 
                                0.0, 
                                DrivetrainSubsystem.PathFollowingConstants.Forward.Right.kd,
                                DrivetrainSubsystem.PathFollowingConstants.Forward.Right.kv,
                                DrivetrainSubsystem.PathFollowingConstants.Forward.Right.ka);

    }
    
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
        leftSpeed = rightFollower.calculate(Robot.drive.getRawLeftEncoder());
        rightSpeed = leftFollower.calculate(Robot.drive.getRawRightEncoder());
      }
      else {
        leftSpeed = leftFollower.calculate(Robot.drive.getRawLeftEncoder());
        rightSpeed = rightFollower.calculate(Robot.drive.getRawRightEncoder());
        
      }
      
      double heading = Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle());
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
      if(this.robotStartedBackwards) {
        if(isReversePath) {

        }
        else {
          desiredHeading = Pathfinder.boundHalfDegrees(desiredHeading + 180.0);
        }
      }
      else {
        if(isReversePath) {
          desiredHeading = Pathfinder.boundHalfDegrees(desiredHeading + 180.0);
        }
        else {

        }
      }

      double headingDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      double turn = DrivetrainSubsystem.PathFollowingConstants.kp_gyro * headingDiff;
      
      SmartDashboard.putNumber("PATH HEADING DIFF", headingDiff);

      log("Heading Diff", headingDiff);
      log("Right Actual", (Robot.drive.motors[0].getSelectedSensorVelocity() * (10.0 / (double) DrivetrainSubsystem.TICKS_PER_REV)) * Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER);
      log("Left Actual", (Robot.drive.motors[3].getSelectedSensorVelocity() * (10.0 / (double) DrivetrainSubsystem.TICKS_PER_REV)) * Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER);


      double left = leftSpeed + turn;
      double right = rightSpeed - turn;
      
      double normalizer = Math.max(Math.max(Math.abs(left), Math.abs(right)), 1.0);

      left /= normalizer;
      right /= normalizer;

      // if(rightFollower.getSegment().acceleration > 0.0 && rightFollower.getSegment().velocity < 1.5) {
      //   left = Math.signum(left) * Math.max(Math.abs(left), 0.6);
      //   right = Math.signum(right) * Math.max(Math.abs(right), 0.6);

      //   Robot.drive.rawDrive(left, right);

      // }
      // else {
      //   Robot.drive.rawDrive(left, right);
      // }

      Robot.drive.rawDrive(left, right);

      log("Left Power", left);
      log("Right Power", right);

      logger.write();

    }
  }

  public boolean isMPFinish(){
    return leftFollower.isFinished() && rightFollower.isFinished();
  }
}
