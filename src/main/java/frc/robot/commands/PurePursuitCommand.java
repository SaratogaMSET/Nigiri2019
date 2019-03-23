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
import com.team254.lib.trajectory.Spline;
import com.team254.lib.trajectory.TrajectoryFollower;
import com.team254.lib.trajectory.Trajectory.Segment;
import com.team319x649.trajectory.FishyPath;
import com.team319x649.trajectory.FishyPathGenerator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.RobotPose;
import frc.robot.RobotMap.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.FishyMath;


public class PurePursuitCommand extends FishyCommand {
    Spline path;

    public static final double kP_gyro_doubletraction = -22.0;
    public static final double kP_gyro_omnitraction = -12.0;

    boolean isPathFinished = false;

    int LAST_CLOSEST_POINT = 0;
    Double spline_x[];
    Double spline_y[];

    // CONSTANTS
    public static final int NUM_SAMPLES = 100;
    public static final double LOOKAHEAD_DISTANCE = 2.5; // feet
    public static final double SPLINE_MAX_VEL = 6.0; // feet/sec




    public PurePursuitCommand(Spline path) {
        this.path = path;
    }

    // The command MUST implement this method - the fields which you want to log
    protected String[] getLogFields() {
        // velocities for MP
        return new String[] {"X", "Y", "H", "Goal X", "GOAL Y", "Goal H", "Left AV", "Right AV", "Left TV", "Right TV", "Heading Diff"};
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        configurePath();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double[] gp = getGoalPoint();
        double leftToRightRatio = getLeftToRightVelocityRatio(getCurvature(gp), DrivetrainSubsystem.EMPIRICAL_WHEELBASE_FEET);
        if(leftToRightRatio < 0.0) {
            throw new RuntimeException("INVALID LEFT/RIGHT RATIO [NEGATIVE]: " + leftToRightRatio);
        }
        double left, right;
        if(leftToRightRatio > 1.0) {
            left = SPLINE_MAX_VEL * Robot.oi.driver.getDriverVertical();
            right = (left/leftToRightRatio);
        }
        else {
            right = SPLINE_MAX_VEL * Robot.oi.driver.getDriverVertical();
            left = (right * leftToRightRatio);
        }

        Robot.drive.motors[0].set(ControlMode.Velocity, FishyMath.rpm2talonunits(FishyMath.fps2rpm(right)));
        Robot.drive.motors[3].set(ControlMode.Velocity, FishyMath.rpm2talonunits(FishyMath.fps2rpm(left)));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return isPathFinished;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        logger.drain();
        logger.flush();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

    public void configurePath() {
        Robot.drive.changeBrakeCoast(false);
        LAST_CLOSEST_POINT = 0;
        for (double i = 0.0; i <= (1.0 + 1.0/(double) NUM_SAMPLES); i += 1.0/(double) NUM_SAMPLES) {
            double[] xy = path.getXandY(i);
            spline_x[(int) Math.round((double) NUM_SAMPLES * i)] = xy[0];
            spline_y[(int) Math.round((double) NUM_SAMPLES * i)] = xy[1];
            // System.out.println(path.calculateLength(i));
            // System.out.println(path.calculateLength());
        }
        
    }

    public double[] getGoalPoint() {
        double[] goal = new double[3];
        double mindist = Double.POSITIVE_INFINITY;
        for(int i = LAST_CLOSEST_POINT; i < NUM_SAMPLES; i++) {
            double d = LOOKAHEAD_DISTANCE - Math.hypot(spline_x[i] - RobotPose.getX(), spline_y[i] - RobotPose.getY());
            if(d > 0.0 && d <= mindist) {
                mindist = d;
                goal = new double[] {spline_x[i], spline_y[i], -d + LOOKAHEAD_DISTANCE};
                LAST_CLOSEST_POINT = i;
            }
        }
        return goal;
    }

    public static double getCurvature(double[] goal) {
        double ROBOT_HEADING = RobotPose.getHeading();
        double a = -Math.tan(ROBOT_HEADING);
        double c = Math.tan(ROBOT_HEADING) * RobotPose.getX() - RobotPose.getY();
        double x = Math.abs(a * goal[0] + goal[1] + c)/Math.sqrt(a*a + 1);
        double curvature = 2 * x / (goal[2] * goal[2]);
        double side = Math.signum(Math.sin(ROBOT_HEADING) * (goal[0] - RobotPose.getX()) - Math.cos(ROBOT_HEADING) * (goal[1] - RobotPose.getY()));
        return side * curvature;
    }

    public static double getLeftToRightVelocityRatio(double curvature, double wheelbase) {
        return (2.0 + curvature * wheelbase)/(2.0 - curvature * wheelbase);
    }
}
