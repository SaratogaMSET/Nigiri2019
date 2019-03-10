package com.team319x649;

import static java.util.Arrays.asList;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;
import com.team319x649.trajectory.*;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.FishyMath;

public class FishyPathCreator extends AbstractFishyPathCreator {

    public static double robotWidthIn = 34.0;
	public static double robotLengthIn = 36.0;

	// This point and points like it can be used when there are common starting locatons for the robot
	// Remember that paths should be generated from the center point of the robot
	private static Waypoint startingPoint = new Waypoint(5.5, 17.354167, 0, 0, 0);
	private TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
    
    public static void main(String[] args) {
		// System.out.println(FishyMath.fps2rpm(12.0));
		new FishyPathCreator().generatePaths();
	}
	
	private FishyPathCreator() {
		config.dt = 0.02;
		config.max_acc = 8.0; // Maximum acceleration in FPS^2
		config.max_vel = 12.0; // Maximum velocity in FPS
		config.max_jerk = 2.0;
	}

    @Override
    protected List<FishyPath> getPaths() {
		List<FishyPath> paths = new ArrayList<>();
		paths.addAll(getConfigArcs());
		paths.addAll(generateTeamPaths());
        return paths;
	}

	/**
	 * Use this method to generate team paths. You can create more methods like this one to organize your path, 
	 * just make sure to add the method call to the returned list in getArcs()
	 * @return the list of team paths to generate
	 */
	private List<FishyPath> generateTeamPaths() {
		 // Create a path with the name of "Example", this will generate a file named ExampleArc
		 FishyPath exampleArc = new FishyPath(config, "FarRocketLeft", DrivetrainSubsystem.WHEELBASE_FEET);
		 // Set the first point to the starating point, this be done with any of the addWaypoint methods
		 // positive X is forward, positive Y is left, units are in feet and degrees
		 exampleArc.addWaypoint(startingPoint);
		 // Add the next point that 4.5 ft forward, and doesn't turn, it also has a max speed of 4 FPS, 
		 // it will arrive at this location going 4 FPS
		 exampleArc.addWaypointRelative(4.5, 0, 0, 3, 3);
		 // Add the next point to be at global coordinates
		 exampleArc.addWaypoint(25, 24, 0, 0, 11.5);
		//  exampleArc.addWaypointRelative(-3.3, 0.675, -29.5, 0, 10.0);
		 
		 return asList(exampleArc); // return asList(path1, path2, path3, ...);
	}
	
	
	/**
	 * Generate the configuration arcs, distance, turning, and speed
	 * DistanceScaling - This path will run 3 feet forward and stop. To tune this
	 * adjust the scaling factor until the robot stops at exactly 3 feet.
	 * TurnScaling - This path will run 3 feet forward and 3 feet to the left, this will 
	 * end at 90 degrees. This path can be used when tuning your heading loop for arc mode.
	 * SpeedTesting - This path will drive 3 feet forward and 3 feet to the left at 3 FPS,
	 * then drive another 3 feed forward and 3 feet to the left. This path will end with 
	 * the robot 6 feet to the left of it's starting position facing the oppostite direction.
	 */
	private List<FishyPath> getConfigArcs() {
		FishyPath distanceScaling = new FishyPath(config, "DistanceScaling", 2.23);
		distanceScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		distanceScaling.addWaypointRelative(5, 0, 0, 0, 10);

		FishyPath turnScaling = new FishyPath(config, "TurnScaling", 2.23);
		turnScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		turnScaling.addWaypointRelative(10, -10, 89.99, 0, 8);


		FishyPath speedTesting = new FishyPath(config, "SpeedTesting", 2.23);
		speedTesting.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		speedTesting.addWaypointRelative(3, 3, 89.99, 1, 3);
		speedTesting.addWaypointRelative(-3, 3, 89.99, 0, 1);

		return asList(distanceScaling, turnScaling, speedTesting);
	}
}