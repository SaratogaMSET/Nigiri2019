package com.team254.lib.trajectory;

import com.team254.lib.trajectory.Trajectory.Segment;
import com.team254.lib.trajectory.TrajectoryGenerator.Config;
import com.team254.lib.trajectory.TrajectoryGenerator.Strategy;

import frc.robot.util.FishyMath;

/**
 * Generate a smooth Trajectory from a Path.
 *
 * @author Art Kalb
 * @author Stephen Pinkerton
 * @author Jared341
 */
public class PathGenerator {
	/**
	 * Generate a path for autonomous driving.
	 *
	 * @param waypoints The waypoints to drive to (FOR THE "GO LEFT" CASE!!!!)
	 * @param config Trajectory config.
	 * @param wheelbase_width Wheelbase separation; units must be consistent with
	 * config and waypoints.
	 * @param name The name of the new path.  THIS MUST BE A VALID JAVA CLASS NAME
	 * @return The path.
	 */
	public static Path makePath(WaypointSequence waypoints,
			TrajectoryGenerator.Config config, double wheelbase_width,
			String name) {
		return new Path(name, makeLeftAndRightTrajectories(generateFromPath(waypoints, config), wheelbase_width));
	}

	static Trajectory.Pair generateLeftAndRightFromSeq(WaypointSequence path,
			TrajectoryGenerator.Config config, double wheelbase_width) {
	  return makeLeftAndRightTrajectories(generateFromPath(path, config),
			  wheelbase_width);
	}

	static Trajectory generateFromPath(WaypointSequence path, TrajectoryGenerator.Config config) {
		if (path.getNumWaypoints() < 2) {
			return null;
		}

		// Compute the total length of the path by creating splines for each pair
		// of waypoints.
		Spline[] splines = new Spline[path.getNumWaypoints() - 1];
		double[] spline_lengths = new double[splines.length];
		for (int i = 0; i < splines.length; ++i) {
			splines[i] = new Spline();
			if (!Spline.reticulateSplines(path.getWaypoint(i), path.getWaypoint(i + 1), splines[i],
					Spline.QuinticHermite)) {
				System.out.println("COULDN'T RETICULATE SPLINE!!");
				return null;
			}
			spline_lengths[i] = splines[i].calculateLength();
		}

		// Generate a smooth trajectory over the total distance.
		Config segmentConfig = new Config();
		segmentConfig.dt = config.dt;
		segmentConfig.max_acc = path.getWaypoint(1).maxAccel;
		segmentConfig.max_vel = path.getWaypoint(1).maxVelocity;
		segmentConfig.max_jerk = config.max_jerk;
		Trajectory traj = TrajectoryGenerator.generate(
			segmentConfig, 
			TrajectoryGenerator.AutomaticStrategy, 
			0.0, 
			0.0, 
			path.getWaypoint(0).theta,
			spline_lengths[0], 
			path.getWaypoint(1).endVelocity, 
			path.getWaypoint(1).theta,
			path.getWaypoint(1).isReverse);
		double distance = spline_lengths[0];
		// System.out.println("DISTANCE SEG 0: " + distance);
		for (int i = 2; i < path.num_waypoints_; ++i) {
			segmentConfig.max_acc = path.getWaypoint(i).maxAccel;
			segmentConfig.max_vel = path.getWaypoint(i).maxVelocity;

			distance = spline_lengths[i - 1];
			double startPos = traj.getSegment(traj.getNumSegments() - 1).pos + traj.getSegment(traj.getNumSegments() - 1).vel * 0;
			
			// System.out.println("DISTANCE SEGMENT " + (i-1) + ": " + distance);
			traj.append(
				TrajectoryGenerator.generate(
					segmentConfig, 
					TrajectoryGenerator.AutomaticStrategy, 
					startPos,
					Math.abs(traj.getSegment(traj.getNumSegments() - 1).vel),
					traj.getSegment(traj.getNumSegments() - 1).heading,
					startPos + distance, 
					path.getWaypoint(i).endVelocity, 
					path.getWaypoint(i).theta,
					path.getWaypoint(i).isReverse));
		}

		// Assign headings based on the splines.
		int cur_spline = 0;
		double cur_spline_start_pos = 0;
		double length_of_splines_finished = 0;
		for (int i = 0; i < traj.getNumSegments(); ++i) {
			double cur_pos = Math.abs(traj.getSegment(i).pos);

			boolean found_spline = false;
			while (!found_spline) {
				// System.out.println(cur_pos);
				// System.out.println(cur_spline_start_pos);
				double cur_pos_relative = Math.abs(cur_pos - cur_spline_start_pos);
				// System.out.println(cur_pos_relative);

				if (cur_pos_relative <= spline_lengths[cur_spline]) {
					double percentage = splines[cur_spline].getPercentageForDistance(cur_pos_relative);
					// System.out.println("PCT: \t" + percentage + "%%%");
					traj.getSegment(i).heading = splines[cur_spline].angleAt(percentage);
					double[] coords = splines[cur_spline].getXandY(percentage);
					traj.getSegment(i).x = coords[0];
					traj.getSegment(i).y = coords[1];
					found_spline = true;
				} else if (cur_spline < splines.length - 1) {
					length_of_splines_finished += spline_lengths[cur_spline];
					cur_spline_start_pos = length_of_splines_finished;
					++cur_spline;
				} else {
					traj.getSegment(i).heading = splines[splines.length - 1].angleAt(1.0);
					double[] coords = splines[splines.length - 1].getXandY(1.0);
					traj.getSegment(i).x = coords[0];
					traj.getSegment(i).y = coords[1];
					found_spline = true;
				}
			}
		}

		// Fix headings so they are continuously additive
		
		double lastUncorrectedHeading = traj.getSegment(0).heading;
		double lastCorrectedHeading = traj.getSegment(0).heading;
		for (int i = 1; i < traj.getNumSegments(); ++i) {
			Segment currentSegment = traj.getSegment(i);
			double uncorrectedHeading = currentSegment.heading;

			double headingDelta = 0;

			if (lastUncorrectedHeading < 0 && uncorrectedHeading > 0  && lastUncorrectedHeading < -Math.PI / 2) {
				headingDelta = -(2 * Math.PI - Math.abs(lastUncorrectedHeading) - Math.abs(uncorrectedHeading));
			} else if (lastUncorrectedHeading > 0 && uncorrectedHeading < 0 && lastUncorrectedHeading > Math.PI / 2) {
				headingDelta = 2 * Math.PI - Math.abs(lastUncorrectedHeading) - Math.abs(uncorrectedHeading);
			} else {
				headingDelta = lastUncorrectedHeading - uncorrectedHeading;
			}

			double correctedHeading = lastCorrectedHeading - headingDelta;
			currentSegment.heading = correctedHeading;
			lastUncorrectedHeading = uncorrectedHeading;
			lastCorrectedHeading = correctedHeading;
		}

		// Reverse headings
		for(Segment s : traj.segments_) {
			s.heading = FishyMath.boundThetaNegPiToPi(s.heading + (s.isReverse ? Math.PI : 0.0));
		}

		for(int i = 1; i < traj.getNumSegments(); i++) {
			Segment currentSegment = traj.getSegment(i);
			Segment previousSegment = traj.getSegment(i - 1);
			
			if(Math.abs(FishyMath.boundThetaNegPiToPi(previousSegment.heading - currentSegment.heading)) >= 0.5 * Math.PI) {
				currentSegment.heading = FishyMath.boundThetaNegPiToPi(currentSegment.heading + Math.PI);
			}
		}


		return traj;
	}

	/**
	 * Generate left and right wheel trajectories from a reference.
	 *
	 * @param input The reference trajectory.
	 * @param wheelbase_width The center-to-center distance between the left and
	 * right sides.
	 * @return [0] is left, [1] is right
	 */
	static Trajectory.Pair makeLeftAndRightTrajectories(Trajectory input,
			double wheelbase_width) {
	  Trajectory[] output = new Trajectory[2];
	  output[0] = input.copy();
	  output[1] = input.copy();
	  Trajectory left = output[0];
	  Trajectory right = output[1];

	  for (int i = 0; i < input.getNumSegments(); ++i) {
		Trajectory.Segment current = input.getSegment(i);
		double cos_angle = Math.cos(current.heading);
		double sin_angle = Math.sin(current.heading);

		Trajectory.Segment s_left = left.getSegment(i);
		s_left.x = current.x - wheelbase_width / 2 * sin_angle;
		s_left.y = current.y + wheelbase_width / 2 * cos_angle;
		if (i > 0) {
		  // Get distance between current and last segment
		  double dist = Math.signum(current.vel) * Math.sqrt((s_left.x - left.getSegment(i - 1).x)
				  * (s_left.x - left.getSegment(i - 1).x)
				  + (s_left.y - left.getSegment(i - 1).y)
				  * (s_left.y - left.getSegment(i - 1).y));
		  s_left.pos = left.getSegment(i - 1).pos + dist;
		  s_left.vel = dist / s_left.dt;
		  s_left.acc = (s_left.vel - left.getSegment(i - 1).vel) / s_left.dt;
		  s_left.jerk = (s_left.acc - left.getSegment(i - 1).acc) / s_left.dt;
		}

		Trajectory.Segment s_right = right.getSegment(i);
		s_right.x = current.x + wheelbase_width / 2 * sin_angle;
		s_right.y = current.y - wheelbase_width / 2 * cos_angle;
		if (i > 0) {
		  // Get distance between current and last segment
		  double dist = Math.signum(current.vel) * Math.sqrt((s_right.x - right.getSegment(i - 1).x)
				  * (s_right.x - right.getSegment(i - 1).x)
				  + (s_right.y - right.getSegment(i - 1).y)
				  * (s_right.y - right.getSegment(i - 1).y));
		  s_right.pos = right.getSegment(i - 1).pos + dist;
		  s_right.vel = dist / s_right.dt;
		  s_right.acc = (s_right.vel - right.getSegment(i - 1).vel) / s_right.dt;
		  s_right.jerk = (s_right.acc - right.getSegment(i - 1).acc) / s_right.dt;
		}
	  }
	  return new Trajectory.Pair(output[0], input, output[1]);
	}

	private static boolean almostEqual(double x, double y) {
		return Math.abs(x - y) < 1E-6;
	}
  }
