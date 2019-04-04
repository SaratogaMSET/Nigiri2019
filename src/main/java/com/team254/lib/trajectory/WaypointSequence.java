package com.team254.lib.trajectory;

import com.team254.lib.util.ChezyMath;

/**
 * A WaypointSequence is a sequence of Waypoints. #whatdidyouexpect
 *
 * @author Art Kalb
 * @author Stephen Pinkerton
 * @author Jared341
 */
public class WaypointSequence {

	public static class Waypoint {

		public Waypoint(double x, double y, double theta, double endVelocity, double maxVelocity) {
			this(x, y, theta, endVelocity, maxVelocity, false);
		}

		public Waypoint(double x, double y, double theta, double endVelocity, double maxVelocity, boolean isReverse) {
			this.x = x;
			this.y = y;
			this.theta = theta;
			this.endVelocity = endVelocity;
			this.maxVelocity = maxVelocity;
			this.isReverse = isReverse;
		}

		public Waypoint(Waypoint tocopy) {
			this.x = tocopy.x;
			this.y = tocopy.y;
			this.theta = tocopy.theta;
			this.endVelocity = tocopy.endVelocity;
			this.maxVelocity = tocopy.maxVelocity;
			this.isReverse = tocopy.isReverse;
		}

		public Waypoint(Waypoint tocopy, double endVelocity, double maxVelocity) {
			this(tocopy, endVelocity, maxVelocity, false);
		}

		public Waypoint(Waypoint tocopy, double endVelocity, double maxVelocity, boolean isReverse) {
			this.x = tocopy.x;
			this.y = tocopy.y;
			this.theta = tocopy.theta;
			this.endVelocity = endVelocity;
			this.maxVelocity = maxVelocity;
			this.isReverse = isReverse;
		}

		public Waypoint(double x, double y, double theta) {
			this.x = x;
			this.y = y;
			this.theta = theta;
			this.endVelocity = 0;
			this.maxVelocity = 0;
			this.isReverse = false;
		}

		public double x;
		public double y;
		public double theta;
		public double endVelocity;
		public double maxVelocity;
		public boolean isReverse;
	}

	Waypoint[] waypoints_;
	int num_waypoints_;

	public WaypointSequence(int max_size) {
		waypoints_ = new Waypoint[max_size];
	}

	public void addWaypoint(Waypoint w) {
		if (num_waypoints_ < waypoints_.length) {
			waypoints_[num_waypoints_] = w;
			++num_waypoints_;
		}
	}

	public int getNumWaypoints() {
		return num_waypoints_;
	}

	public Waypoint getWaypoint(int index) {
		if (index >= 0 && index < getNumWaypoints()) {
			return waypoints_[index];
		} else {
			return null;
		}
	}

}
