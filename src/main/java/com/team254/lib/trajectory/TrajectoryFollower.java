package com.team254.lib.trajectory;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.FishyMath;

/**
 * PID + Feedforward controller for following a Trajectory.
 *
 * @author Jared341
 */
public class TrajectoryFollower {

	private double kp_;
	private double kd_;
	private double last_error_;
	private double current_heading = 0;
	private int current_segment;
	private Trajectory profile_;
	private double starting_distance_feet_;

	public TrajectoryFollower() {
	}

	public TrajectoryFollower(Trajectory t) {
		profile_ = t;
	}

	public void configure(double kp, double kd, double offset_feet) {
		kp_ = kp;
		kd_ = kd;
		starting_distance_feet_ = offset_feet;
	}

	public void reset() {
		last_error_ = 0.0;
		current_segment = 0;
	}

	public void setTrajectory(Trajectory profile) {
		profile_ = profile;
	}

	public double calculateTargetRPM(double position_feet) {
		double trajectory_pos = position_feet - starting_distance_feet_;
		if (current_segment < profile_.getNumSegments()) {
			Trajectory.Segment segment = profile_.getSegment(current_segment);
			double errorFeet = segment.pos - trajectory_pos;
			double errorRotations = errorFeet / (Math.PI * DrivetrainSubsystem.WHEEL_DIAMETER);
			double targetRPM = kp_ * errorRotations + kd_ * ((errorRotations - last_error_) / segment.dt - FishyMath.fps2rpm(segment.vel)) + FishyMath.fps2rpm(segment.vel);

			last_error_ = errorRotations;
			current_heading = segment.heading;
			current_segment++;
			// System.out.println("so far: " + distance_so_far + "; output: " + output);
			return targetRPM;
		} else {
			return 0;
		}
	}

	public double getHeading() {
		return current_heading;
	}

	public Trajectory.Segment getSegment() {
		return profile_.segments_[current_segment];
	}

	public boolean isFinishedTrajectory() {
		return current_segment >= profile_.getNumSegments();
	}
}
