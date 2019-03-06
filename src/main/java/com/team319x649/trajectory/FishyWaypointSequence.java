package com.team319x649.trajectory;

import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.util.ChezyMath;

/**
 * Just 254's WaypointSequence, but JSONable
 * 
 * @author ttremblay
 */
public class FishyWaypointSequence extends WaypointSequence {

	public static class FishyWaypoint extends Waypoint {

		public FishyWaypoint(double x, double y, double theta, double endVelocity, double maxVelocity) {
			super(x, y, theta, endVelocity, maxVelocity);
		}

		public FishyWaypoint(FishyWaypoint tocopy) {
			super(tocopy);
		}

		public double x;
		public double y;
		public double theta;
	}

	FishyWaypoint[] waypoints_;
	int num_waypoints_;

	public FishyWaypointSequence(int max_size) {
		super(max_size);
	}

	public void addWaypoint(FishyWaypoint w) {
		if (num_waypoints_ < waypoints_.length) {
			waypoints_[num_waypoints_] = w;
			++num_waypoints_;
		}
	}

	public int getNumWaypoints() {
		return num_waypoints_;
	}

	public FishyWaypoint getWaypoint(int index) {
		if (index >= 0 && index < getNumWaypoints()) {
			return waypoints_[index];
		} else {
			return null;
		}
	}

	public FishyWaypointSequence invertY() {
		FishyWaypointSequence inverted = new FishyWaypointSequence(waypoints_.length);
		inverted.num_waypoints_ = num_waypoints_;
		for (int i = 0; i < num_waypoints_; ++i) {
			inverted.waypoints_[i] = waypoints_[i];
			inverted.waypoints_[i].y *= -1;
			inverted.waypoints_[i].theta = ChezyMath
					.boundAngle0to2PiRadians(2 * Math.PI - inverted.waypoints_[i].theta);
		}

		return inverted;
	}
}
