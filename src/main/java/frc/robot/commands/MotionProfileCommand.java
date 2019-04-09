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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotMap.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.ControlLoop;
import frc.robot.util.FishyMath;


public class MotionProfileCommand extends FishyCommand {
  Path path;
  TrajectoryFollower leftFollower, rightFollower;

  boolean isPathFinished;
  boolean robotStartedBackwards;

  double previous_heading = 0.0;

  public static final double kP_gyro_doubletraction = -10.0;
  public static final double kP_gyro_omnitraction = -10.0;

  double heading_offset = 0.0;

  private ControlLoop motionProfileControlLoop = new ControlLoop(0.05) {
    @Override
    public boolean init() {
      return true;
    }

    @Override
    public void loop() {
      followPath();
    }

    @Override
    public void end() {

    }

    private void followPath(){
      if(leftFollower.isFinishedTrajectory() && rightFollower.isFinishedTrajectory()){
        Robot.drive.isPathRunning = false;
        motionProfileControlLoop.stopLoop();
        Robot.drive.rawDrive(0.0, 0.0);
      } else {
        log("dt", motionProfileControlLoop.getDt());

        log("Right Target", rightFollower.getSegment().vel);
        log("Left Target", leftFollower.getSegment().vel);

        log("Left TPos", leftFollower.getSegment().pos);
        log("Right TPos", rightFollower.getSegment().pos);
        log("LX", leftFollower.getSegment().x);
        log("LY", leftFollower.getSegment().y);

        log("RX", rightFollower.getSegment().x);
        log("RY", rightFollower.getSegment().y);

        double leftSpeedRPM, rightSpeedRPM;

        leftSpeedRPM = leftFollower.calculateTargetRPM(Robot.drive.getLeftEncoderDistance());
        rightSpeedRPM = rightFollower.calculateTargetRPM(Robot.drive.getRightEncoderDistance());

        double heading = -FishyMath.boundThetaNeg180to180(Robot.gyro.getGyroAngle());
        double desiredHeading = FishyMath.boundThetaNeg180to180(FishyMath.r2d(leftFollower.getHeading()));
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

        double canTimeStart = Timer.getFPGATimestamp();
        Robot.drive.motors[0].set(ControlMode.Velocity, FishyMath.rpm2talonunits(right));
        Robot.drive.motors[3].set(ControlMode.Velocity, FishyMath.rpm2talonunits(left));
        double canTimeEnd = Timer.getFPGATimestamp();

        log("Left Setpoint", left);
        log("Right Setpoint", right);
        log("Heading Diff", headingDiff);
        log("Target Heading", desiredHeading);
        log("Actual Heading", heading);
        log("T %", turn/left);
        log("Left APos", Robot.drive.getLeftEncoderDistance());
        log("Right APos", Robot.drive.getRightEncoderDistance());
        log("CAN Time(ms)", (canTimeEnd - canTimeStart)*1000.0);

        logger.write();

      }
    }
  };

  public MotionProfileCommand(String pathName) {
    this(pathName, false, 0.0);
  }

  public MotionProfileCommand(String pathName, boolean robotStartedBackwards) {
    this(pathName, robotStartedBackwards, 0.0);
  }

  public MotionProfileCommand(String pathName, double heading_offset) {
    this(pathName, false, heading_offset);
  }

  public MotionProfileCommand(String pathName, boolean robotStartedBackwards, double heading_offset) {
    this();
    path = FishyPathGenerator.importPath(pathName);
    this.robotStartedBackwards = robotStartedBackwards;
    this.heading_offset = heading_offset;
  }

  private MotionProfileCommand() {
    isPathFinished = false;
    new Thread(motionProfileControlLoop).start();
  }

  // The command MUST implement this method - the fields which you want to log
  protected String[] getLogFields() {
    // velocities for MP
    return new String[] {"dt", "CAN Time(ms)", "Left TPos", "Right TPos", "Left APos", "Right APos", "Right Target", "Left Target", "Left Actual", "Right Actual", "Left Setpoint", "Right Setpoint", "Heading Diff", "Target Heading", "Acutal Heading", "T %", "LX", "LY", "RX", "RY"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(path != null) {
      leftFollower = new TrajectoryFollower(path.getLeftTrajectory());
      rightFollower = new TrajectoryFollower(path.getRightTrajectory());
      configurePath();
      motionProfileControlLoop.startPeriodic();
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
    return Robot.isManualAuto || isPathFinished || (leftFollower.isFinishedTrajectory() && rightFollower.isFinishedTrajectory());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.motors[0].set(ControlMode.PercentOutput, 0.0);
    Robot.drive.motors[3].set(ControlMode.PercentOutput, 0.0);
    logger.drain();
    logger.flush();
    Robot.gyro.gyro.setAngleAdjustment(0.0);
    motionProfileControlLoop.stopLoop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
  // i think we should run a pure pursuit

  public void configurePath() {
    Robot.drive.changeBrakeCoast(false);

    leftFollower.configure(80.0, 0.0, Robot.drive.getLeftEncoderDistance());
    rightFollower.configure(80.0, 0.0, Robot.drive.getRightEncoderDistance());

    Robot.gyro.gyro.setAngleAdjustment(heading_offset);

    previous_heading = FishyMath.boundThetaNegPiToPi(leftFollower.getHeading());
  }
}
