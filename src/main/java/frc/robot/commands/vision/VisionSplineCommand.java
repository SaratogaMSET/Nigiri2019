package frc.robot.commands.vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.FishyCommand;
import frc.robot.commands.PurePursuitCommand;
import jaci.pathfinder.Pathfinder;


public class VisionSplineCommand extends FishyCommand {
  Timer time;
  PurePursuitCommand purePursuitCommand;

  public VisionSplineCommand() {
    time = new Timer();
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
    Double distance = Robot.vision.getDistance();
    Double angle = Robot.vision.getAngleDisplacement();

    // if(angleSet || angle == null || (distance != null && distance < 50.0)) {

    // }
    // else {
    //   angleSet = true;
    //   SmartDashboard.putNumber("VISION ANGLE", angle);
    //   gyroHoldCommand.start();
    //   gyroHoldCommand.setTargetAngle(Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle() + angle));
    //   System.out.println(Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle() + angle));

    // }

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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
