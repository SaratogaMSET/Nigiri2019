package com.team319x649;

import static java.util.Arrays.asList;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;
import com.team319x649.trajectory.*;

import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

import frc.robot.util.FishyMath;

public class FishyPathCreator extends AbstractFishyPathCreator {
// i think we should run a pure pursuit
    public static double robotWidthIn = 34.0;
	public static double robotLengthIn = 36.0;
	public static TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();

	// Starting points
	private static Waypoint HAB1L = new Waypoint(5.5, 17.354167, 0, 0, 0);

    public static void main(String[] args) {
		new FishyPathCreator().generatePaths();
		System.out.println("HELLo");
	}

	private FishyPathCreator() {
		// DO NOT TOUCH
		config.dt = 0.05;
		config.max_acc = 10.0; // Robot max acceleration in FPS^2
		config.max_vel = 14.0; // Robot max velocity in FPS
		config.max_jerk = 100.0;
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
		return asList(getDoubleRocketFast(), getDoubleRocketFast2(), getDoubleRocketFast3());
		// return asList(getHAB1LxROCKLFxLOADLxROCKLF());

		// return asList(getMidCargoLeft(), getMidCargoToLSL(), getLSLToCargoNear());

		// return asList(getMidCargoLeftSlow(), getMidCargoToLSLSlow(), getLSLToCargoNearSlow(), 
		// getMidCargoLeft(), getMidCargoToLSL(), getLSLToCargoNear(), getDoubleRocketFast(), getDoubleRocketFast2(), getDoubleRocketFast3());

		// getNearRocketRight(), getNearRocketToLoadingStation()); // return asList(path1, path2, path3, ...);
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
		FishyPath straightSlowShort = new FishyPath(config, "StrightSlowShort", 2.23);
		straightSlowShort.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		straightSlowShort.addWaypointRelative(5, 0, 0, 0, 4);

		FishyPath straightSlowLong = new FishyPath(config, "StrightSlowLong", 2.23);
		straightSlowLong.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		straightSlowLong.addWaypointRelative(15, 0, 0, 0, 4, 10);

		FishyPath straightFastShort = new FishyPath(config, "StraightFastShort", 2.23);
		straightFastShort.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		straightFastShort.addWaypointRelative(5, 0, 0, 0, 10);

		FishyPath straightFastLong = new FishyPath(config, "StraightFastLong", DrivetrainSubsystem.WHEELBASE_FEET);
		straightFastLong.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		straightFastLong.addWaypointRelative(10, 0, 0, 0, 10, 10, false);

		FishyPath straightSlowShortRevese = new FishyPath(config, "StraightSlowShortRevese", 2.23);
		straightSlowShortRevese.addWaypoint(new Waypoint(2, 13.5, Math.PI, 0, 0));
		straightSlowShortRevese.addWaypointRelative(5, 0, 0, 0, 4, true);

		FishyPath straightSlowLongReverse = new FishyPath(config, "StraightSlowLongReverse", 2.23);
		straightSlowLongReverse.addWaypoint(new Waypoint(2, 13.5, Math.PI, 0, 0));
		straightSlowLongReverse.addWaypointRelative(20, 0, 0, 0, 4, true);

		FishyPath straightFastShortReverse = new FishyPath(config, "StraightFastShortReverse", 2.23);
		straightFastShortReverse.addWaypoint(new Waypoint(2, 13.5, Math.PI, 0, 0));
		straightFastShortReverse.addWaypointRelative(5, 0, 0, 0, 10, true);

		FishyPath straightFastLongReverse = new FishyPath(config, "StraightFastLongReverse", 2.23);
		straightFastLongReverse.addWaypoint(new Waypoint(2, 13.5, Math.PI, 0, 0));
		straightFastLongReverse.addWaypointRelative(20, 0, 0, 0, 10, true);


		FishyPath turnScaling = new FishyPath(config, "TurnScaling", 2.23);
		turnScaling.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		turnScaling.addWaypointRelative(10, -10, -89.99, 0, 8);

		FishyPath turnScalingReverse = new FishyPath(config, "TurnScalingReverse", 2.23);
		turnScalingReverse.addWaypoint(new Waypoint(2, 13.5, Math.PI, 0, 0));
		turnScalingReverse.addWaypointRelative(10, -10, -89.99, 0, 8, true);

		FishyPath speedTesting = new FishyPath(config, "SpeedTesting", 2.23);
		speedTesting.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		speedTesting.addWaypointRelative(3, 3, 89.99, 1, 3);
		speedTesting.addWaypointRelative(-3, 3, 89.99, 0, 1);

		FishyPath headingTesting = new FishyPath(config, "HeadingTesting", 2.23);
		headingTesting.addWaypoint(20.0, 20.0, -135);
		headingTesting.addWaypoint(10.0, 10.0, -150, 0, 10);

		return asList(straightFastLong, straightSlowLong, turnScaling);
	}


	/***********************    TEAM PATHS    *********************************************/

	private FishyPath getIanAssistPathLeft() {
		FishyPath ianAssistLeft = new FishyPath(config, "IanAssistRocketLeft", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssistLeft.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		ianAssistLeft.addWaypointRelative(4.5, 0, 0, 6, 6, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssistLeft.addWaypoint(25, 22.8, 0, 0, 11.4, true);
		return ianAssistLeft;
	}

	private FishyPath getIanAssistPathRight() {
		FishyPath ianAssistRight = new FishyPath(config, "IanAssistRocketRight", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssistRight.addWaypoint(new Waypoint(5.5, 27 - 17.354167, 0, 0, 0));
		ianAssistRight.addWaypointRelative(4.5, 0, 0, 6, 6, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssistRight.addWaypoint(25, 27 - 22.8, 0, 0, 11.4, true);
		return ianAssistRight;
	}

	private FishyPath getIanAssistPathRightTurn() {
		FishyPath ianAssistRight = new FishyPath(config, "IanAssistRocketRightTurn", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssistRight.addWaypoint(new Waypoint(5.5, 27 - 17.354167, 0, 0, 0));
		ianAssistRight.addWaypointRelative(4.5, 0, 0, 6, 6, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssistRight.addWaypoint(25, 27 - 22.8, 20, 0, 8, true);
		return ianAssistRight;
	}

	private FishyPath getSlowIanAssistPathLeft() {
		FishyPath ianAssistLeft = new FishyPath(config, "SlowIanAssistRocketLeft", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssistLeft.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		ianAssistLeft.addWaypointRelative(4.5, 0, 0, 4, 4, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssistLeft.addWaypoint(25, 22.8, 0, 0, 4, true);
		return ianAssistLeft;
	}

	private FishyPath getSlowIanAssistPathRight() {
		FishyPath ianAssistRight = new FishyPath(config, "SlowIanAssistRocketRight", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssistRight.addWaypoint(new Waypoint(5.5, 27 - 17.354167, 0, 0, 0));
		ianAssistRight.addWaypointRelative(4.5, 0, 0, 4, 4, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssistRight.addWaypoint(25, 27 - 22.8, 0, 0, 4, true);
		return ianAssistRight;
	}


	// private FishyPath getMidCargoLeft() {
	// 	FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL2", DrivetrainSubsystem.WHEELBASE_FEET);
	// 	leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
	// 	leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 6, 6, true);
	// 	// leftNearCargoShip.addWaypointRelative(7, 0.65, 15, 6, 8.5, true, true);
	// 	// leftNearCargoShip.addWaypoint(21.7, 22, 89.99, 0, 6, true, true);
	// 	leftNearCargoShip.addWaypoint(24, 20, 0, 0, 8.5, true);
	// 	return leftNearCargoShip;
	// }

	
	private FishyPath getMidCargoLeft() {
		FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL2", DrivetrainSubsystem.WHEELBASE_FEET);

		leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 4, 4, true);
		leftNearCargoShip.addWaypoint(17, 18, -160, 7, 7, true);
		leftNearCargoShip.addWaypoint(24.5, 22, -120, 0, 7, true);
		leftNearCargoShip.addWaypoint(22.8, 17.5, -91, 0, 3, false);

		return leftNearCargoShip;
	}

	private FishyPath getMidCargoToLSL() {
		FishyPath leftNearCargoShip = new FishyPath(config, "CL2-LSL", DrivetrainSubsystem.WHEELBASE_FEET);
		leftNearCargoShip.addWaypoint(23.2, 17.5, -91, 0, 0);

		leftNearCargoShip.addWaypoint(25.5, 22.5, -150, 0, 4, true);
		leftNearCargoShip.addWaypoint(6, 25.5, -180, 4, 7, false);
		leftNearCargoShip.addWaypoint(1, 25.5, -180, 0, 4, false);

		return leftNearCargoShip;
	}

	private FishyPath getLSLToCargoNear() {
		FishyPath leftNearCargoShip = new FishyPath(config, "LSL-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		leftNearCargoShip.addWaypoint(1, 25.5, -180, 0, 0);
		leftNearCargoShip.addWaypoint(8, 25.5, -180, 6, 6, true);
		// leftNearCargoShip.addWaypoint(17, 20, -180, 8, 9, true);
		leftNearCargoShip.addWaypoint(23.5, 23.5, -140, 0, 6, true);
		leftNearCargoShip.addWaypoint(22, 17.5, -91, 0, 4, false);
		return leftNearCargoShip;
	}

	private FishyPath getMidCargoLeftSlow() {
		FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL2-Slow", DrivetrainSubsystem.WHEELBASE_FEET);

		leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 4, 4, true);
		leftNearCargoShip.addWaypoint(17, 18, -160, 4, 4, true);
		leftNearCargoShip.addWaypoint(24.5, 22, -120, 0, 4, true);
		leftNearCargoShip.addWaypoint(22.8, 17.5, -91, 0, 3, false);

		return leftNearCargoShip;
	}

	private FishyPath getMidCargoToLSLSlow() {
		FishyPath leftNearCargoShip = new FishyPath(config, "CL2-LSL-Slow", DrivetrainSubsystem.WHEELBASE_FEET);
		leftNearCargoShip.addWaypoint(23.2, 17.5, -91, 0, 0);

		leftNearCargoShip.addWaypoint(25.5, 22.5, -150, 0, 4, true);
		leftNearCargoShip.addWaypoint(6, 25.5, -180, 4, 4, false);
		leftNearCargoShip.addWaypoint(1, 25.5, -180, 0, 4, false);

		return leftNearCargoShip;
	}

	private FishyPath getLSLToCargoNearSlow() {
		FishyPath leftNearCargoShip = new FishyPath(config, "LSL-CL1-Slow", DrivetrainSubsystem.WHEELBASE_FEET);
		leftNearCargoShip.addWaypoint(1, 25.5, -180, 0, 0);
		leftNearCargoShip.addWaypoint(8, 25.5, -180, 4, 4, true);
		// leftNearCargoShip.addWaypoint(17, 20, -180, 8, 9, true);
		leftNearCargoShip.addWaypoint(23.5, 23.5, -140, 0, 4, true);
		leftNearCargoShip.addWaypoint(22, 17.5, -91, 0, 4, false);
		return leftNearCargoShip;
	}



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
		HAB1LxROCKLF.addWaypointRelative(4.5, 0, 0, 3, 3, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		// Add the next point to be at global coordinates
		HAB1LxROCKLF.addWaypoint(25, 23.8, 0, 0, 11, true);
		// Relative coordinates
		HAB1LxROCKLF.addWaypointRelative(-4.1, 1.0, -29.5, 4.0, 4.0, false);

		return HAB1LxROCKLF;
	}

	/**
	 * Two-hatch far-rocket-left auto: Starts at the left of HAB1, goes to far rocket left, then loading station, then far rocket left again
	 */
	private FishyPath getHAB1LxROCKLFxLOADLxROCKLF() {
		FishyPath twoHatchRocketLeft = new FishyPath(config, "HAB1LxROCKLFxLOADLxROCKLF", DrivetrainSubsystem.WHEELBASE_FEET);
		twoHatchRocketLeft.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		twoHatchRocketLeft.addWaypointRelative(4.5, 0, 0, 6, 6, 12, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		// twoHatchRocketLeft.addWaypoint(4 - 2.2 + 17.5, 24.2 - 3.2, 180, 5.0, 10.0, true);
		twoHatchRocketLeft.addWaypoint(26, 24.5, -150, 0, 12, 8, true);

		twoHatchRocketLeft.addWaypoint(21.627, 23.9, 180 - 31, 0, 4, 8, false);
		// 21.5, 24.35
		twoHatchRocketLeft.addWaypointRelative(4, 0.0, 30.0 + 30, 0, 4.0, 8, true);

		twoHatchRocketLeft.addWaypoint(6, 25.433, 180, 2.0, 11.0, 10, false);
		twoHatchRocketLeft.addWaypoint(4.5-3.2, 25.433, 180, 0.0, 2.0, false);

		// twoHatchRocketLeft.addWaypoint(4 - 2.2 + 17.5, 24.2 - 3.2, 180, 4.0, 10.0, true);
		// twoHatchRocketLeft.addWaypoint(4 - 2.2 + 17.5 + 6.0, 23.7, 180 - 10, 0.0, 4.0, true);
		// twoHatchRocketLeft.addWaypoint(22, 24.7, 180-30, 0, 4, false);

		// */

		return twoHatchRocketLeft;
	}

	private FishyPath getDoubleRocketFast() {
		FishyPath twoHatchRocketLeft = new FishyPath(config, "DoubleRocketFast", DrivetrainSubsystem.WHEELBASE_FEET);
		twoHatchRocketLeft.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		twoHatchRocketLeft.addWaypointRelative(4.5, 0, 0, 4, 4, true);
		twoHatchRocketLeft.addWaypoint(22.942, 23.822, 60, 0, 11, true);

		return twoHatchRocketLeft;
	}

	private FishyPath getDoubleRocketFast2() {
		FishyPath path = new FishyPath(config, "DoubleRocketFast2", DrivetrainSubsystem.WHEELBASE_FEET);
		path.addWaypoint(new Waypoint(22.87, 23.78, FishyMath.d2r(150), 0, 0));
		path.addWaypointRelative(-1.3, 0.75, 0, 0, 3, 8, false);
		path.addWaypoint(22.87, 23.78, 150, 0, 3, 8, true);

		return path;
	}

	private FishyPath getDoubleRocketFast3() {
		FishyPath path = new FishyPath(config, "DoubleRocketFast3", DrivetrainSubsystem.WHEELBASE_FEET);
		path.addWaypoint(new Waypoint(22.87, 23.78, FishyMath.d2r(-150), 0, 0));
		path.addWaypoint(3, 24.78, 180, 4, 10, 11, false);
		// path.addWaypoint(-1.5, 16.3, 180, 0, 4, 10, false);
		// path.addWaypoint(22.87, 22.38, 180, 0, 11, 11, true);

		return path;
	}
}

		// private FishyPath getMidCargoLeft() {
		// 	FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL2", DrivetrainSubsystem.WHEELBASE_FEET);
	
		// 	leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		// 	leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 4, 4, true);
		// 	leftNearCargoShip.addWaypoint(17, 18, -170, 6, 6, true);
		// 	leftNearCargoShip.addWaypoint(24.5, 22, -120, 0, 6, true);
		// 	leftNearCargoShip.addWaypoint(23.12, 17.5, -91, 0, 3, false);
	
		// 	return leftNearCargoShip;
		// }
	
		// private FishyPath getMidCargoToLSL() {
		// 	FishyPath leftNearCargoShip = new FishyPath(config, "CL2-LSL", DrivetrainSubsystem.WHEELBASE_FEET);
		// 	leftNearCargoShip.addWaypoint(23.12, 17.5, -91, 0, 0);
	
		// 	leftNearCargoShip.addWaypoint(25.5, 22.5, -150, 0, 4, true);
		// 	leftNearCargoShip.addWaypoint(6, 25, -180, 4, 6, false);
		// 	leftNearCargoShip.addWaypoint(1, 25, -180, 0, 4, false);
	
		// 	return leftNearCargoShip;
		// }
	
		// private FishyPath getLSLToCargoNear() {
		// 	FishyPath leftNearCargoShip = new FishyPath(config, "LSL-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		// 	leftNearCargoShip.addWaypoint(1, 25, -180, 0, 0);
		// 	leftNearCargoShip.addWaypoint(8, 25, -180, 6, 6, true);
		// 	// leftNearCargoShip.addWaypoint(17, 20, -180, 8, 9, true);
		// 	leftNearCargoShip.addWaypoint(23.5, 23.5, -150, 0, 6, true);
		// 	leftNearCargoShip.addWaypoint(21, 17.5, -91, 0, 4, false);
		// 	return leftNearCargoShip;
		// }