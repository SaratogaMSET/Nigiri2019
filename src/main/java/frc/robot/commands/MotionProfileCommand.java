/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryFollower;
import com.team319x649.trajectory.FishyPath;
import com.team319x649.trajectory.FishyPathGenerator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.FishyMath;
import jaci.pathfinder.*;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;


public class MotionProfileCommand extends FishyCommand {
  Path path;
  Notifier followerNotifier;
  TrajectoryFollower leftFollower, rightFollower;

  double left_encoder_count, right_encoder_count;
  boolean isReversePath;

  boolean isPathFinished;

  public MotionProfileCommand(FishyPath fishyPath, double left_encoder_count, double right_encoder_count) {
    this();
    path = fishyPath.getPath();
  }

  public MotionProfileCommand(String pathName) {
    this();
    path = FishyPathGenerator.importPath(pathName);
  }

  private MotionProfileCommand() {
    isPathFinished = false;
    followerNotifier = new Notifier(this::followPath);
    isReversePath = false;
  }

  public void setPathDirection(boolean isReverse) {
    this.isReversePath = isReverse;
  }

  // The command MUST implement this method - the fields which you want to log
  protected String[] getLogFields() {
    // velocities for MP
    return new String[] {"Right Target", "Left Target", "Left Actual", "Right Actual", "Left Power", "Right Power", "Heading Diff"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(path != null) {
      leftFollower = new TrajectoryFollower(path.getLeftTrajectory());
      rightFollower = new TrajectoryFollower(path.getRightTrajectory());
      configurePath();
      followerNotifier.startPeriodic(0.02);
    }
    else {
      SmartDashboard.putBoolean("MP PATH FAIL", true);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(leftFollower == null) {
      return isPathFinished;
    }
    return isPathFinished || (leftFollower.isFinishedTrajectory() && rightFollower.isFinishedTrajectory());
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

  public void configurePath() {
    Robot.drive.changeBrakeCoast(false);
    leftFollower.configure(10.0, 0.0, Robot.drive.getLeftEncoderDistance());
    rightFollower.configure(10.0, 0.0, Robot.drive.getRightEncoderDistance());

    // if(reversePath) {
    //   leftFollower.configureEncoder(Robot.drive.getRawRightEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
    //   rightFollower.configureEncoder(Robot.drive.getRawLeftEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);

    // }
    // else {
    //   leftFollower.configureEncoder(Robot.drive.getRawLeftEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
    //   rightFollower.configureEncoder(Robot.drive.getRawRightEncoder(), DrivetrainSubsystem.TICKS_PER_REV, DrivetrainSubsystem.WHEEL_DIAMETER);
    // }
    
    // if(reversePath) {
    //   leftFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.kp, 
    //                             0.0, 
    //                             DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.kd,
    //                             DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.kv,
    //                             DrivetrainSubsystem.PathFollowingConstants.Reverse.Left.ka);

    //   rightFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.kp, 
    //                             0.0, 
    //                             DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.kd,
    //                             DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.kv,
    //                             DrivetrainSubsystem.PathFollowingConstants.Reverse.Right.ka);

    // }
    // else {
    //   leftFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Forward.Left.kp, 
    //                             0.0, 
    //                             DrivetrainSubsystem.PathFollowingConstants.Forward.Left.kd,
    //                             DrivetrainSubsystem.PathFollowingConstants.Forward.Left.kv,
    //                             DrivetrainSubsystem.PathFollowingConstants.Forward.Left.ka);

    //   rightFollower.configurePIDVA(DrivetrainSubsystem.PathFollowingConstants.Forward.Right.kp, 
    //                             0.0, 
    //                             DrivetrainSubsystem.PathFollowingConstants.Forward.Right.kd,
    //                             DrivetrainSubsystem.PathFollowingConstants.Forward.Right.kv,
    //                             DrivetrainSubsystem.PathFollowingConstants.Forward.Right.ka);

    // }
    
    // this.isReversePath = reversePath;

    // followerNotifier = new Notifier(this::followPath);
    // followerNotifier.startPeriodic(leftTraj.get(0).dt);
  }

  private void followPath(){
    if(leftFollower.isFinishedTrajectory() && rightFollower.isFinishedTrajectory()){
      Robot.drive.isPathRunning = false;
      followerNotifier.stop();
      followerNotifier = null;
    } else {
      log("Right Target", rightFollower.getSegment().vel);
      log("Left Target", leftFollower.getSegment().vel);

      double leftSpeedRPM, rightSpeedRPM;
      if(isReversePath) {
        leftSpeedRPM = rightFollower.calculateTargetRPM(Robot.drive.getLeftEncoderDistance());
        rightSpeedRPM = leftFollower.calculateTargetRPM(Robot.drive.getRightEncoderDistance());
      }
      else {
        leftSpeedRPM = leftFollower.calculateTargetRPM(Robot.drive.getLeftEncoderDistance());
        rightSpeedRPM = rightFollower.calculateTargetRPM(Robot.drive.getRightEncoderDistance());
        
      }
      
      double heading = Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle());
      double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
      // if(this.robotStartedBackwards) {
      //   if(isReversePath) {

      //   }
      //   else {
      //     desiredHeading = Pathfinder.boundHalfDegrees(desiredHeading + 180.0);
      //   }
      // }
      // else {
      //   if(isReversePath) {
      //     desiredHeading = Pathfinder.boundHalfDegrees(desiredHeading + 180.0);
      //   }
      //   else {

      //   }
      // }

      double headingDiff = Pathfinder.boundHalfDegrees(desiredHeading - heading);
      double turn = DrivetrainSubsystem.PathFollowingConstants.kp_gyro * headingDiff;
      
      log("Heading Diff", headingDiff);
      log("Right Actual", (Robot.drive.motors[0].getSelectedSensorVelocity() * (10.0 / (double) DrivetrainSubsystem.TICKS_PER_REV)) * Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER);
      log("Left Actual", (Robot.drive.motors[3].getSelectedSensorVelocity() * (10.0 / (double) DrivetrainSubsystem.TICKS_PER_REV)) * Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER);



      double left = leftSpeedRPM;
      double right = rightSpeedRPM;
      
      Robot.drive.motors[0].set(ControlMode.Velocity, FishyMath.rpm2talonunits(left));
      Robot.drive.motors[3].set(ControlMode.Velocity, FishyMath.rpm2talonunits(right));

      log("Left Setpoint", left);
      log("Right Setpoint", right);

      logger.write();

    }
  }
}
