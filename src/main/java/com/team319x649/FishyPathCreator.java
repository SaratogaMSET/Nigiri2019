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
	public static TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();

	// Starting points
	private static Waypoint HAB1L = new Waypoint(5.5, 17.354167, 0, 0, 0);

    public static void main(String[] args) {
		new FishyPathCreator().generatePaths();
	}

	private FishyPathCreator() {
		// DO NOT TOUCH
		config.dt = 0.02;
		config.max_acc = 8.0; // Robot max acceleration in FPS^2
		config.max_vel = 12.0; // Robot max velocity in FPS
		config.max_jerk = 0.0;
	}

    @Override
    protected List<FishyPath> getPaths() {
		List<FishyPath> paths = new ArrayList<>();
		// paths.addAll(getConfigArcs());
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
		 exampleArc.addWaypoint(HAB1L);
		 // Add the next point that 4.5 ft forward, and doesn't turn, it also has a max speed of 4 FPS,
		 // it will arrive at this location going 4 FPS
		 exampleArc.addWaypointRelative(4.5, 0, 0, 3, 3);
		 // Add the next point to be at global coordinates
		 exampleArc.addWaypoint(25, 24, 0, 0, 11.5);
		//  exampleArc.addWaypointRelative(-3.3, 0.675, -29.5, 0, 10.0);

		FishyPath frontCargo = new FishyPath(config, "HABM-CF", DrivetrainSubsystem.WHEELBASE_FEET);
		frontCargo.addWaypoint(5.5, 14.5, 0, 0, 0);
		//frontCargo.addWaypointRelative(10.5, 0, 0, 0, 11.5);
		frontCargo.addWaypointRelative(4, 0, 0, 3, 3);
    		frontCargo.addWaypoint(16.25, 14.5, 0, 0, 6);


		FishyPath nearCargoShip = new FishyPath(config, "HAB1L-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		nearCargoShip.addWaypoint(HAB1L);
		nearCargoShip.addWaypointRelative(4, 0, 0, 3, 3);
		nearCargoShip.addWaypointRelative(6, 3, 25, 11.5, 11.5);
		nearCargoShip.addWaypoint(21.7, 18, 89.99, 0, 11.5);

		FishyPath frontToLoadingStation1 = new FishyPath(config, "CLF-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		frontToLoadingStation1.addWaypoint(16, 14.5, 0, 0, 0);
		frontToLoadingStation1.addWaypointRelative(-3, 3, 89.99, 2, 3);
		frontToLoadingStation1.addWaypoint(16, 20.5, 179.98, 0, 3);

		FishyPath frontToLoadingStation2 = new FishyPath(config, "CLF-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		frontToLoadingStation2.addWaypoint(16, 20.5, 0, 0, 0);
		frontToLoadingStation2.addWaypoint(1.5, 24.75, 0, 0, 11.5);

		FishyPath nearRocketLeftToLoading1 = new FishyPath(config, "RLL-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		nearRocketLeftToLoading1.addWaypoint(15.5, 24.25, 30, 0, 0);
		nearRocketLeftToLoading1.addWaypoint(10, 18, 89.99, 0, 11.5);

		FishyPath nearRocketLeftToLoading2 = new FishyPath(config, "RLL-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		nearRocketLeftToLoading2.addWaypoint(10, 18, 89.99, 0, 0);
		nearRocketLeftToLoading2.addWaypoint(1.5, 24.75, 0, 0, 11.5);

		FishyPath loadingToNearRocketLeft1 = new FishyPath(config, "LSL-RLL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToNearRocketLeft1.addWaypoint(1.5, 24.75, 0, 0, 0);
		loadingToNearRocketLeft1.addWaypoint(10, 18, 89.99, 0, 11.5);

		FishyPath loadingToNearRocketLeft2 = new FishyPath(config, "LSL-RLL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToNearRocketLeft2.addWaypoint(10, 18, 89.99, 0, 0);
		loadingToNearRocketLeft2.addWaypoint(15.5, 24.25, 30, 0, 11.5);

		FishyPath loadingToMidCargoShip1 = new FishyPath(config, "LSL-CL2-1", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToMidCargoShip1.addWaypoint(1.5, 24.75, 0, 0, 0);
		loadingToMidCargoShip1.addWaypointRelative(15, -5, 15, 10, 11.5);
		loadingToMidCargoShip1.addWaypoint(25, 24, 30, 0, 11.5);

		FishyPath loadingToMidCargoShip2 = new FishyPath(config, "LSL-CL2-2", DrivetrainSubsystem.WHEELBASE_FEET);
		loadingToMidCargoShip2.addWaypoint(25, 24, 30, 0, 0);
		loadingToMidCargoShip2.addWaypoint(23.5, 18, 89.99, 0, 11.5);

		FishyPath cargoShipLeft1ToLoading1 = new FishyPath(config, "CL1-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		cargoShipLeft1ToLoading1.addWaypoint(21.75, 18, 89.99, 0, 0);
		cargoShipLeft1ToLoading1.addWaypoint(25, 24, 30, 0, 11.5);

		FishyPath cargoShipLeft1ToLoading2 = new FishyPath(config, "CL1-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		cargoShipLeft1ToLoading2.addWaypoint(25, 24, 0, 0, 0);
		cargoShipLeft1ToLoading2.addWaypointRelative(-8.5, -4.25, -15, 10, 11.5);
		cargoShipLeft1ToLoading2.addWaypoint(4.5, 24.75, 0, 0, 11.5);

		FishyPath straightPath = new FishyPath(config, "straight", DrivetrainSubsystem.WHEELBASE_FEET);
		straightPath.addWaypoint(10, 20, 0, 0, 0);
		straightPath.addWaypoint(24, 20, 0, 0, 11.5);

		FishyPath revPath = new FishyPath(config, "rev", DrivetrainSubsystem.WHEELBASE_FEET);
		revPath.addWaypoint(24, 20, 0, 0, 0);
		revPath.addWaypoint(10, 20, 0, 0, 11.5);

		FishyPath anglePath = new FishyPath(config, "angle", DrivetrainSubsystem.WHEELBASE_FEET);
		anglePath.addWaypoint(24, 24, 45, 0, 0);
		anglePath.addWaypoint(15, 15, 45, 0, 11.5);

		return asList(exampleArc, frontCargo, nearCargoShip, frontToLoadingStation1, frontToLoadingStation2, loadingToNearRocketLeft1, loadingToNearRocketLeft2, loadingToMidCargoShip1, loadingToMidCargoShip2, cargoShipLeft1ToLoading1, cargoShipLeft1ToLoading2, straightPath, revPath, anglePath, nearRocketLeftToLoading1, nearRocketLeftToLoading2 ); // return asList(path1, path2, path3, ...);
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


	/***********************    TEAM PATHS    *********************************************/

	/**
	 * Single-hatch far-rocket-left auto: Starts at the left of HAB1, goes to far rocket left.
	 */
	private FishyPath getHAB1LxROCKLF() {
		// Create a path with the name of "HAB1LxROCKLF", this will generate three files named HAB1LxROCKLF-(right/left/center).csv
		FishyPath HAB1LxROCKLF = new FishyPath(config, "HAB1LxROCKLF", DrivetrainSubsystem.WHEELBASE_FEET);
		// Set the first point to the starting point, this can only be done with the ABSOLUTE addWaypoint methods
		// positive X is forward, positive Y is left, CCW is positive heading, CW is negative heading - NOTE: THIS IS THE OPPOSITE OF THE NAVX SYSTEM
		// units are in feet and degrees
		HAB1LxROCKLF.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		// Add the next point that 4.5 ft forward, and doesn't turn, it also has a max speed of 3 ft/s,
		// it will arrive at this location going 3 ft/s
		// The robot is travelling backwards
		// The robot is facing opposite positive X (reference frame)
		// Relative coordinates
		HAB1LxROCKLF.addWaypointRelative(4.5, 0, 0, 3, 3, true, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		// Add the next point to be at global coordinates
		HAB1LxROCKLF.addWaypoint(25, 23.5, 0, 0, 11.7, true, true);
		// Relative coordinates
		HAB1LxROCKLF.addWaypointRelative(-4.1, 1.0, -29.5, 4.0, 4.0, false, true);

		return HAB1LxROCKLF;
	}

	/**
	 * Two-hatch far-rocket-left auto: Starts at the left of HAB1, goes to far rocket left, then loading station, then far rocket left again
	 */
	private FishyPath getHAB1LxROCKLFxLOADLxROCKLF() {
		// Create a path with the name of "HAB1LxROCKLF", this will generate three files named HAB1LxROCKLF-(right/left/center).csv
		FishyPath HAB1LxROCKLF = new FishyPath(config, "HAB1LxROCKLFxLOADLxROCKLF", DrivetrainSubsystem.WHEELBASE_FEET);
		HAB1LxROCKLF.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		// Add the next point that 4.5 ft forward, and doesn't turn, it also has a max speed of 3 ft/s,
		// it will arrive at this location going 3 ft/s
		// The robot is travelling backwards
		// The robot is facing opposite positive X (reference frame)
		// Relative coordinates
		HAB1LxROCKLF.addWaypointRelative(4.5, 0, 0, 3, 3, true, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		// Add the next point to be at global coordinates
		HAB1LxROCKLF.addWaypoint(25, 24, 0, 0, 11.7, true, true);
		// Relative coordinates
		HAB1LxROCKLF.addWaypointRelative(-3.3, 0.8, -29.5, 4.0, 4.0, false, true);
		HAB1LxROCKLF.addWaypointRelative(-2.61, 1.477, 0, 0.0, 4.0, false, true);

		HAB1LxROCKLF.addWaypointRelative(2, 0.0, 29.5 + 45, 0, 11.7, false, true);
		HAB1LxROCKLF.addWaypointRelative(-5, -2.5, -45, 11.7, 11.7, false, true);
		HAB1LxROCKLF.addWaypoint(1.5, 24.8, 0, 0.0, 11.3, false, true);
		HAB1LxROCKLF.addWaypoint(25, 24, 0, 0, 11.7, true, true);



		return HAB1LxROCKLF;
	}
}
