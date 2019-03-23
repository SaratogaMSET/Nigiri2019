package frc.robot.commands.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import com.team254.lib.trajectory.Spline;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotPose;
import frc.robot.commands.FishyCommand;
import frc.robot.commands.PurePursuitCommand;
import frc.robot.util.FishyMath;
import frc.robot.util.RigidTransform2d;
import frc.robot.util.Rotation2d;
import frc.robot.util.Translation2d;
import java.util.HashMap;

public class VisionSplineCommand extends FishyCommand {
  Timer time;
  public PurePursuitCommand purePursuitCommand;
  boolean isFinished = false;

  Map<Translation2d, Rotation2d> visionTargetHeadings = new HashMap<>();

  public VisionSplineCommand() {
    time = new Timer();
    visionTargetHeadings = new HashMap<>();
    // Loading station
    visionTargetHeadings.put(new Translation2d(1.5, 24.8), Rotation2d.fromDegrees(180));
    visionTargetHeadings.put(new Translation2d(1.5, 2.2), Rotation2d.fromDegrees(180));
    // Near rockets
    visionTargetHeadings.put(new Translation2d(16.7, 24.7), Rotation2d.fromDegrees(29.8));
    visionTargetHeadings.put(new Translation2d(16.7, 2.3), Rotation2d.fromDegrees(-29.8));
    // Center rockets
    visionTargetHeadings.put(new Translation2d(19.2, 3.8), Rotation2d.fromDegrees(-90.0));
    visionTargetHeadings.put(new Translation2d(19.2, 23.2), Rotation2d.fromDegrees(90.0));
    // Far rockets
    visionTargetHeadings.put(new Translation2d(21.8, 2.3), Rotation2d.fromDegrees(-150.2));
    visionTargetHeadings.put(new Translation2d(21.8, 27-2.3), Rotation2d.fromDegrees(150.2));
  }

  @Override
  protected String[] getLogFields() {
    return new String[] {"Theta", "X", "Y", "Skew"};
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    time.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Activate vision target hold
    Robot.vision.readData();

    if(purePursuitCommand != null && purePursuitCommand.isRunning()) {

    }
    else {
      // Make sure up-to-date data
      if(Robot.vision.received_timestamp != null && Robot.vision.received_timestamp + 0.5 > Timer.getFPGATimestamp()) {
        double dx = Robot.vision.delta_x;
        double dy = Robot.vision.delta_y;

        RigidTransform2d robot = RobotPose.getPose();
        double robotx = robot.getTranslation().x();
        double roboty = robot.getTranslation().y();
        double roboth = FishyMath.boundThetaNeg180to180(robot.getRotation().getDegrees());

        RigidTransform2d goal = RobotPose.getPose().transformBy(new RigidTransform2d(new Translation2d(dy, -dx), Rotation2d.fromDegrees(0.0)));
        goal.setRotation(Rotation2d.fromDegrees(FishyMath.boundThetaNeg180to180(180)));

        double goalx = goal.getTranslation().x();
        double goaly = goal.getTranslation().y();
        double goalh = FishyMath.boundThetaNeg180to180(goal.getRotation().getDegrees());

        Spline visionSpline = new Spline();
        Spline.reticulateSplines(robotx, roboty, FishyMath.d2r(roboth), goalx, goaly, FishyMath.d2r(goalh), visionSpline, Spline.QuinticHermite);

        System.out.println("SPLINE GENERATING...");
        System.out.println(visionSpline.toString());

        purePursuitCommand = new PurePursuitCommand(visionSpline);
        purePursuitCommand.start();

      }
      else {
        Robot.gyro.driverGyroPID.setSetpoint(FishyMath.boundThetaNeg180to180(Robot.gyro.getGyroAngle() + Robot.oi.driver.getDriverHorizontal() * 20.0));
        Robot.gyro.driverGyroPID.enable();
        Robot.drive.driveFwdRotate(Robot.oi.driver.getDriverVertical(), Robot.gyro.driverPIDOutput);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // Let Command be canceled manually
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // gyroHoldCommand.cancel();
    // SmartDashboard.putNumber("Left Encoder", Robot.drive.getLeftEncoder());
    // SmartDashboard.putNumber("Right Encoder", Robot.drive.getRightEncoder());
    purePursuitCommand.cancel();
    purePursuitCommand.close();
    purePursuitCommand = null;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
