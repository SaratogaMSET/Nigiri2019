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
import com.team254.lib.trajectory.Trajectory.Segment;
import com.team319x649.trajectory.FishyPath;
import com.team319x649.trajectory.FishyPathGenerator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.FishyMath;


public class MotionProfileCommand extends FishyCommand {
  Path path;
  Notifier followerNotifier;
  TrajectoryFollower leftFollower, rightFollower;

  boolean isReversePath;
  boolean isPathFinished;

  public static final double kP_gyro_doubletraction = -4.0;
  public static final double kP_gyro_omnitraction = -0.5;

  public MotionProfileCommand(FishyPath fishyPath) {
    this();
    path = fishyPath.getPath();
  }

  public MotionProfileCommand(String pathName) {
    this(pathName, false);
  }

  public MotionProfileCommand(String pathName, boolean isReversePath) {
    this();
    path = FishyPathGenerator.importPath(pathName);
    setPathDirection(isReversePath);
  }

  private MotionProfileCommand() {
    isPathFinished = false;
  }

  public void setPathDirection(boolean isReverse) {
    this.isReversePath = isReverse;
    if(isReverse) {
      for(Segment s : path.getLeftTrajectory().getSegments()) {
        s.vel *= -1;
        s.pos *= -1;
        s.jerk *= -1;
        s.acc *= -1;
      }
      for(Segment s : path.getTrajectory().getSegments()) {
        s.vel *= -1;
        s.pos *= -1;
        s.jerk *= -1;
        s.acc *= -1;
      }
      for(Segment s : path.getRightTrajectory().getSegments()) {
        s.vel *= -1;
        s.pos *= -1;
        s.jerk *= -1;
        s.acc *= -1;
      }
    }
  }

  // The command MUST implement this method - the fields which you want to log
  protected String[] getLogFields() {
    // velocities for MP
    return new String[] {"Left TPos", "Right TPos", "Left APos", "Right APos", "Right Target", "Left Target", "Left Actual", "Right Actual", "Left Setpoint", "Right Setpoint", "Heading Diff", "Target Heading", "Acutal Heading", "T %", "LX", "LY", "RX", "RY"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(path != null) {
      leftFollower = new TrajectoryFollower(path.getLeftTrajectory());
      rightFollower = new TrajectoryFollower(path.getRightTrajectory());
      configurePath();
      followerNotifier = new Notifier(this::followPath);
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
    Robot.drive.motors[0].set(ControlMode.Velocity, 0.0);
    Robot.drive.motors[3].set(ControlMode.Velocity, 0.0);
    logger.drain();
    logger.flush();
    if(followerNotifier != null) {
      followerNotifier.stop();
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public void configurePath() {
    Robot.drive.changeBrakeCoast(false);

    if(this.isReversePath) {
      leftFollower.configure(20.0, 0.0, Robot.drive.getRightEncoderDistance());
      rightFollower.configure(20.0, 0.0, Robot.drive.getLeftEncoderDistance());
    }
    else {
      leftFollower.configure(20.0, 0.0, Robot.drive.getLeftEncoderDistance());
      rightFollower.configure(20.0, 0.0, Robot.drive.getRightEncoderDistance());
    }
    
    

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
      // Robot.drive.motors[0].set(ControlMode.Velocity, 0.0);
      // Robot.drive.motors[3].set(ControlMode.Velocity, 0.0);
      Robot.drive.rawDrive(0.0, 0.0);
    } else {
      log("Right Target", rightFollower.getSegment().vel);
      log("Left Target", leftFollower.getSegment().vel);

      log("Left TPos", leftFollower.getSegment().pos);
      log("Right TPos", rightFollower.getSegment().pos);
      log("LX", leftFollower.getSegment().x);
      log("LY", leftFollower.getSegment().y);

      log("RX", rightFollower.getSegment().x);
      log("RY", rightFollower.getSegment().y);

      double leftSpeedRPM, rightSpeedRPM;

      if(this.isReversePath) {
        leftSpeedRPM = rightFollower.calculateTargetRPM(Robot.drive.getLeftEncoderDistance());
        rightSpeedRPM = leftFollower.calculateTargetRPM(Robot.drive.getRightEncoderDistance());
      }
      else {
        leftSpeedRPM = leftFollower.calculateTargetRPM(Robot.drive.getLeftEncoderDistance());
        rightSpeedRPM = rightFollower.calculateTargetRPM(Robot.drive.getRightEncoderDistance());  
      }
      
             
      
      double heading = FishyMath.boundThetaNeg180to180(-Robot.gyro.getGyroAngle());
      double desiredHeading = FishyMath.r2d(leftFollower.getHeading());
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

      double headingDiff = FishyMath.boundThetaNeg180to180(desiredHeading - heading);
      double gyroConstant;
      if(leftFollower.getSegment().acc < 0) {
        gyroConstant = kP_gyro_omnitraction;
      }
      else {
        gyroConstant = kP_gyro_doubletraction;
      }
      double turn = gyroConstant * headingDiff;
      
      log("Right Actual", Robot.drive.getRightEncoderVelocity());
      log("Left Actual", Robot.drive.getLeftEncoderVelocity());

      double left = leftSpeedRPM + turn;
      double right = rightSpeedRPM - turn;
      
      Robot.drive.motors[0].set(ControlMode.Velocity, FishyMath.rpm2talonunits(right));
      Robot.drive.motors[3].set(ControlMode.Velocity, FishyMath.rpm2talonunits(left));

      log("Left Setpoint", left);
      log("Right Setpoint", right);
      log("Heading Diff", headingDiff);
      log("Target Heading", desiredHeading);
      log("Actual Heading", heading);

      log("T %", turn/left);


      log("Left APos", Robot.drive.getLeftEncoderDistance());
      log("Right APos", Robot.drive.getRightEncoderDistance());



      logger.write();

    }
  }
}
