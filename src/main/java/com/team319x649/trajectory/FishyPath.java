package com.team319x649.trajectory;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.TrajectoryGenerator.Config;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;


public class FishyPath {
	private Config config;
	private WaypointSequence waypointSequence;
	private String name;
	private double wheelbase;
	private Path path;

	public FishyPath(Config config, String name, double wheelbaseFeet) {
		this(config, name, wheelbaseFeet, false);
	}

	public FishyPath(Config config, String name, double wheelbaseFeet, boolean driveBackwards) {
		this.name = name;
		this.config = config;
		this.waypointSequence = new WaypointSequence(10);
		this.wheelbase = wheelbaseFeet;
	}

	public FishyPath(FishyPath toCopy) {
		config = toCopy.config;
		waypointSequence = toCopy.waypointSequence;
	}

	public boolean isExportEnabled() {
		return this.isExportEnabled();
	}

	public void setWaypointSequence(WaypointSequence wps) {
		waypointSequence = wps;
	}

	public WaypointSequence getWaypointSequence() {
		return waypointSequence;
	}

	public void addWaypoint(Waypoint wp) {
		addWaypoint(wp, false, false);
	}

	public void addWaypoint(Waypoint wp, boolean isReverse, boolean isFacingReverse) {
		if (wp.maxVelocity == 0.0 && waypointSequence.getNumWaypoints() > 0) {
			getLastWaypoint().endVelocity = config.max_vel;
		}
		this.waypointSequence.addWaypoint(new Waypoint(wp, 0, config.max_vel, isReverse, isFacingReverse));
	}

	public void addWaypointRadians(double x, double y, double theta_rad, double endVelocity, double maxVelocity) {
		addWaypointRadians(x, y, theta_rad, endVelocity, maxVelocity, false, false);
	}

	public void addWaypointRadians(double x, double y, double theta_rad, double endVelocity, double maxVelocity, boolean isReverse, boolean isFacingReverse) {
		this.waypointSequence.addWaypoint(new Waypoint(x, y, theta_rad, endVelocity, maxVelocity, isReverse, isFacingReverse));
	}

	public void addWaypoint(double x, double y, double theta_deg, double endVelocity, double maxVelocity) {
		addWaypoint(x, y, theta_deg, endVelocity, maxVelocity, false, false);
	}

	public void addWaypoint(double x, double y, double theta_deg, double endVelocity, double maxVelocity, boolean isReverse, boolean isFacingReverse) {
		this.waypointSequence.addWaypoint(new Waypoint(x, y, Math.toRadians(theta_deg), endVelocity, maxVelocity, isReverse, isFacingReverse));
	}

	public void addWaypoint(double x, double y, double theta_deg) {
		addWaypoint(x, y, theta_deg, false, false);
	}

	public void addWaypoint(double x, double y, double theta_deg, boolean isReverse, boolean isFacingReverse) {
		if (waypointSequence.getNumWaypoints() > 0) {
			getLastWaypoint().endVelocity = config.max_vel;
		}
		addWaypoint(new Waypoint(x, y, Math.toRadians(theta_deg), 0, config.max_vel, isReverse, isFacingReverse));
	}

	public void addWaypoint(Waypoint wp, double endVelocity, double maxVelocity) {
		addWaypoint(wp, endVelocity, maxVelocity, false, false);
	}

	public void addWaypoint(Waypoint wp, double endVelocity, double maxVelocity, boolean isReverse, boolean isFacingReverse) {
		this.waypointSequence.addWaypoint(new Waypoint(wp, endVelocity, maxVelocity, isReverse, isFacingReverse));
	}

	public void addWaypointRelative(double x, double y, double theta_deg) {
		addWaypointRelative(x, y, theta_deg, false, false);
	}

	public void addWaypointRelative(double x, double y, double theta_deg, boolean isReverse, boolean isFacingReverse) {
		if (waypointSequence.getNumWaypoints() > 1) {
			getLastWaypoint().endVelocity = config.max_vel;
		}
		addWaypointRelative(x, y, theta_deg, 0, config.max_vel, isReverse, isFacingReverse);
	}

	public void addWaypointRelative(double x, double y, double theta_deg, double endVelocity, double maxVelocity) {
		addWaypointRelative(x, y, theta_deg, endVelocity, maxVelocity, false, false);
	}

	public void addWaypointRelative(double x, double y, double theta_deg, double endVelocity, double maxVelocity, boolean isReverse, boolean isFacingReverse) {
		Waypoint lastWaypoint = getLastWaypoint();
		Waypoint newWaypoint = new Waypoint(lastWaypoint.x + x, lastWaypoint.y + y,
				lastWaypoint.theta + Math.toRadians(theta_deg), endVelocity, maxVelocity, isReverse, isFacingReverse);
		this.waypointSequence.addWaypoint(newWaypoint);
	}

	public Waypoint getLastWaypoint() {
		Waypoint lastWaypoint = this.waypointSequence.getWaypoint(this.waypointSequence.getNumWaypoints() - 1);
		return lastWaypoint;
	}

	public void setConfig(Config c) {
		this.config = c;
	}

	public Config getConfig() {
		return this.config;
	}

	public String getName() {
		return name;
	}

	public double getWheelbaseFeet() {
		return wheelbase;
	}

	public Path getPath() {
		if(path == null) {
			return FishyPathGenerator.makePath(this);
		}
		return path;
	}
}
