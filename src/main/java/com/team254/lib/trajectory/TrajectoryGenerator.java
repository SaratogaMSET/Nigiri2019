package com.team254.lib.trajectory;

import com.team254.lib.trajectory.Trajectory.Segment;

/**
 * Factory class for creating Trajectories.
 *
 * @author Jared341
 */
public class TrajectoryGenerator {

	///// INNER CLASSES /////
	public static class Config {

		public double dt;
		public double max_vel;
		public double max_acc;
		public double max_jerk;
	}

	public static Trajectory generate(Config config, double start_vel, double start_pos,
			double goal_pos, double goal_vel, double maxVelocity, double segment_max_accel, boolean isReverse) {

		start_vel = Math.abs(start_vel);

		Trajectory traj;
//		double adjusted_max_vel = maxVelocity;
		double distanceAtMaxVelocity = ((goal_vel * goal_vel) - (start_vel * start_vel)) /
				(2 * segment_max_accel * segment_max_accel)  + (goal_pos - start_pos) / 2;
//		double maxPossibleVelocity = config.max_acc / 2 * Math.sqrt(distanceAtMaxVelocity + (start_vel * start_vel) / (config.max_acc * config.max_acc));
		double maxPossibleVelocity = start_vel + Math.sqrt(2 * segment_max_accel * distanceAtMaxVelocity);
		double adjusted_max_vel = Math.min(maxVelocity, maxPossibleVelocity);
		//		}
		double t_rampup = (adjusted_max_vel - start_vel) / segment_max_accel;
		double x_rampup = (adjusted_max_vel + start_vel) / 2.0 * t_rampup;
		if (x_rampup > goal_pos - start_pos) {
			System.out.println("CAN'T REACH FINAL VELOCITY");
		}
		double t_rampdown = (adjusted_max_vel - goal_vel) / segment_max_accel;
		double x_rampdown = (adjusted_max_vel + goal_vel) / 2.0 * t_rampdown;
		double x_cruise = goal_pos - start_pos - x_rampdown - x_rampup;
		double t_cruise = x_cruise / adjusted_max_vel;

		int time = (int) ((t_rampup + t_rampdown + t_cruise) / config.dt);
		double impulse = (t_rampup + t_cruise) / config.dt;
		traj = generateValues(config.dt, start_vel, start_pos, adjusted_max_vel, segment_max_accel, impulse, time, goal_pos, goal_vel, isReverse);

		return traj;
	}

	private static Trajectory generateValues(double dt, double start_vel, double start_pos,
			double max_vel, double max_accel, double total_impulse, int time, double goalPosition, double goalVel, boolean isReverse) {
		if (time <= 0) {
			return null;
		}

		Trajectory traj = new Trajectory(time);

		double currentPosition = start_pos;
		double currentVelocity = start_vel * ((isReverse) ? -1.0 : 1.0);
		double currentAcceleration = 0;
		System.out.println("Startvel: " + currentVelocity);
		System.out.println("Goalvel: " + goalVel);

		double seg_acceleration = 0.0;
		for (int i = 0; i <= time; i++) {
			Trajectory.Segment current = new Trajectory.Segment();
			current.pos = currentPosition;
			current.vel = currentVelocity;
			current.acc = currentAcceleration;
			current.dt = dt;
			current.isReverse = isReverse;
			traj.setSegment(i, current);




			if (i >= total_impulse) {
				if(isReverse) {
					seg_acceleration = max_accel;
				}
				else {
					seg_acceleration = -max_accel;
				}
				currentVelocity += seg_acceleration * dt;
				currentAcceleration = seg_acceleration;
			} else {
				if(isReverse) {
					seg_acceleration = -max_accel;
				}
				else {
					seg_acceleration = max_accel;
				}
				currentVelocity += seg_acceleration * dt;
				currentAcceleration = seg_acceleration;
			}

			if (currentVelocity >= max_vel) {
				currentVelocity = max_vel;
				currentAcceleration = 0;
			}

			if (currentVelocity <= -max_vel) {
				currentVelocity = -max_vel;
				currentAcceleration = 0;
			}

			double empirical_velocity = currentVelocity;
			if(isReverse) {
				empirical_velocity *= -1;
			}

			currentPosition += empirical_velocity * dt;
		}
		
		System.out.println("Missing distance: " + (goalPosition - currentPosition));
		System.out.println("Endvel: " + currentVelocity);

		return traj;
	}
}
