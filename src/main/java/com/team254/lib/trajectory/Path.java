package com.team254.lib.trajectory;

/**
 * Base class for an autonomous path.
 * 
 * @author Jared341
 */
public class Path {
	protected Trajectory.Pair trajectoryPair;
	protected String name_;

	public Path(String name, Trajectory.Pair newTraj) {
		name_ = name;
		trajectoryPair = newTraj;
	}

	public Path() {

	}

	public String getName() {
		return name_;
	}

	public Trajectory getTrajectory() {
		return trajectoryPair.center;
	}

	public Trajectory getLeftTrajectory() {
		return trajectoryPair.left;
	}

	public Trajectory getRightTrajectory() {
		return trajectoryPair.right;
	}

	public Trajectory.Pair getTrajectoryPair() {
		return trajectoryPair;
	}

	public double getEndHeading() {
		int numSegments = getLeftTrajectory().getNumSegments();
		Trajectory.Segment lastSegment = getLeftTrajectory().getSegment(numSegments - 1);
		return lastSegment.heading;
	}
	
	public void offsetHeading(double theta_rad) {
		getLeftTrajectory().offsetHeading(theta_rad);
		getRightTrajectory().offsetHeading(theta_rad);
	}
}
