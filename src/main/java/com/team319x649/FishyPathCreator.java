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
		config.max_acc = 8.0; // Maximum acceleration in FPS^
		config.max_vel = 12.0; // Maximum velocity in FP
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
		 // Create a path with the name of"Example", this will generate a file named ExampleArc
		 FishyPath exampleArc = new FishyPath(config, "FarRocketLeft", DrivetrainSubsystem.WHEELBASE_FEET);
		 // Set the first point to the starating point, this be done with any of the addWaypoint methods
		 // positive X is forward, positive Y is left, units ae in feet and degrees
		 exampleArc.addWaypoint(startingPoint);
		 // Add the next point that 4.5 ft forward, and doesn't turn, it also has a max speed of 4 FPS, 
		 // it will arrive at this location going 4 FPS
		 exampleArc.addWaypointRelative(4.5, 0, 0, 3, 3);
		 // Add the next point to be at global coordinates
		 exampleArc.addWaypoint(25, 24, 0, 0, 11.5);
		//  exampleArc.addWaypointRelative(-3.3, 0.675, -29.5, 0, 10.0);
		
		FishyPath frontCargo = new FishyPath(config, "HABM-CF", DrivetrainSubsystem.WHEELBASE_FEET);
		frontCargo.addWaypoint(5.5, 14.5, 0, 0, 0);
		frontCargo.addWaypointRelative(10.5, 0, 0, 0, 11.5);

		FishyPath nearCargoShip = new FishyPath(config, "HAB1L-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		nearCargoShip.addWaypoint(startingPoint);
		nearCargoShip.addWaypointRelative(4, 0, 0, 3, 3);
		nearCargoShip.addWaypointRelative(6, 3, 25, 10, 11.5);
		nearCargoShip.addWaypoint(21.7, 18, 89.99, 0, 11.5);

		FishyPath frontToLoadingStation1 = new FishyPath(config, "CLF-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		frontToLoadingStation1.addWaypoint(16, 14.5, 0, 0, 0);
		frontToLoadingStation1.addWaypointRelative(-3, 3, 89.99, 1, 3);
		frontToLoadingStation1.addWaypoint(16, 20.5, 179.98, 0, 3);

		FishyPath frontToLoadingStation2 = new FishyPath(config, "CLF-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		frontToLoadingStation2.addWaypoint(16, 20.5, 0, 0, 0);
		frontToLoadingStation2.addWaypoint(1.5, 24.75, 0, 0, 11.5);
		
		FishyPath loadingToNearRocketLeft1 = new FishyPath(config, "LSL-RLL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToNearRocketLeft1.addWaypoint(1.5, 24.75, 0, 0, 0);
		loadingToNearRocketLeft1.addWaypoint(10, 18, 89.99, 0, 11.5);

		FishyPath loadingToNearRocketLeft2 = new FishyPath(config, "LSL-RLL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToNearRocketLeft2.addWaypoint(10, 18, 89.99, 0, 0);
		loadingToNearRocketLeft2.addWaypoint(14.5, 23.75, 30, 0, 11.5);

		FishyPath loadingToMidCargoShip1 = new FishyPath(config, "LSL-CL2-1", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToMidCargoShip1.addWaypoint(1.5, 24.75, 0, 0, 0);
		loadingToMidCargoShip1.addWaypointRelative(15, -5, 15, 10, 11.5);
		loadingToMidCargoShip1.addWaypoint(25, 24, 30, 0, 11.5);

		FishyPath loadingToMidCargoShip2 = new FishyPath(config, "LSL-CL2-2", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToMidCargoShip2.addWaypoint(25, 24, 30, 0, 0);
		loadingToMidCargoShip2.addWaypoint(23.5, 18, 89.99, 0, 11.5);

		 return asList(exampleArc, frontCargo, nearCargoShip, frontToLoadingStation1, frontToLoadingStation2, loadingToNearRocketLeft1, loadingToNearRocketLeft2, loadingToMidCargoShip1, loadingToMidCargoShip2); // return asList(path1, path2, path3, ...);
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