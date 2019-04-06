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
		// return asList(getSlowIanAssistPathLeft(), getSlowIanAssistPathRight());
		// return asList(getHAB1LxROCKLFxLOADLxROCKLF());

		return asList(getFarCargoLeftSlow(), getFarCargoLeft());
		// return asList(getIanAssistPathLeft(), getIanAssistPathRight(), getFrontCargoPath(), 
		// getCloseFrontPath(), getNearCargoLeft(), getNearCargoRight(), getMidCargoLeft(), getFarCargoRight(), 
		// getNearCargoLeftToLoadingStation(), getNearCargoRightToLoadingStation(), getFarCargoLeftToLoadingStation(), 
		// getFarCargoRightToLoadingStation(), getRocketToLoadingStationLeft(), getNearRocketLeft(), 
		// getNearRocketRight(), getNearRocketToLoadingStation(), getLoadingStationToNearCargoLeft(),
		// getBackRocketDriveInLeft(), getIanAssistPathRightTurn(), getBackRocketDriveInRight(),
		// getFarCargoLeft(), getFarCargoLeftToLSL(), getLSLToCargoMid()); // return asList(path1, path2, path3, ...);
		// return asList(getIanAssistPathLeft(), getIanAssistPathRight(), 
		// getFrontCargoPath(), getCloseFrontPath(), getNearCargoLeft(), 
		// getNearCargoRight(), getRocketToLoadingStationLeft(), getNearRocketLeft(), 
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
		straightSlowLong.addWaypointRelative(20, 0, 0, 0, 4);

		FishyPath straightFastShort = new FishyPath(config, "StraightFastShort", 2.23);
		straightFastShort.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		straightFastShort.addWaypointRelative(5, 0, 0, 0, 10);

		FishyPath straightFastLong = new FishyPath(config, "StraightFastLong", 2.23);
		straightFastLong.addWaypoint(new Waypoint(2, 13.5, 0, 0, 0));
		straightFastLong.addWaypointRelative(20, 0, 0, 0, 10);

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

		return asList(straightSlowShort, straightSlowLong, straightFastShort, straightFastLong, straightSlowShortRevese, straightSlowLongReverse, straightFastShortReverse, straightFastLongReverse, turnScaling, turnScalingReverse, speedTesting, headingTesting);
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

	private FishyPath getFrontCargoPath() {
		FishyPath frontCargo = new FishyPath(config, "HABM-CF", DrivetrainSubsystem.WHEELBASE_FEET);
		frontCargo.addWaypoint(5.5, 14.5, 0, 0, 0);
		frontCargo.addWaypointRelative(4.5, 0, 0, 6, 6);
		frontCargo.addWaypoint(14, 14.5, 0, 0, 7);
		return frontCargo;
	}

	private FishyPath getCloseFrontPath() {
		FishyPath closeFrontCargo = new FishyPath(config, "HABM-CF-close", DrivetrainSubsystem.WHEELBASE_FEET);
		closeFrontCargo.addWaypoint(5.5, 14.5, 0, 0, 0);
		closeFrontCargo.addWaypointRelative(4.5, 0, 0, 6, 6);
		closeFrontCargo.addWaypoint(16, 14.5, 0, 0, 7);
		return closeFrontCargo;
	}

	private FishyPath getNearCargoLeft() {
		FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 6, 6, true);
		// leftNearCargoShip.addWaypointRelative(7, 0.65, 15, 6, 8.5, true, true);
		// leftNearCargoShip.addWaypoint(21.7, 22, 89.99, 0, 6, true, true);
		leftNearCargoShip.addWaypoint(21.7, 20, 0, 0, 8.5, true);
		return leftNearCargoShip;
	}

	private FishyPath getMidCargoLeft() {
		FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL2", DrivetrainSubsystem.WHEELBASE_FEET);
		leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 6, 6, true);
		// leftNearCargoShip.addWaypointRelative(7, 0.65, 15, 6, 8.5, true, true);
		// leftNearCargoShip.addWaypoint(21.7, 22, 89.99, 0, 6, true, true);
		leftNearCargoShip.addWaypoint(24, 20, 0, 0, 8.5, true);
		return leftNearCargoShip;
	}

	private FishyPath getFarCargoLeft() {
		FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL3", DrivetrainSubsystem.WHEELBASE_FEET);
		// twoHatchRocketLeft.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		// twoHatchRocketLeft.addWaypointRelative(4.5, 0, 0, 3, 3, true);

		leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 4, 4, true);
		leftNearCargoShip.addWaypoint(17, 17.5, -180, 8, 10, true);
		leftNearCargoShip.addWaypoint(24, 22, -100, 0, 6, true);
		// leftNearCargoShip.addWaypoint(25, 18, 89, 0, 4, false);

		// leftNearCargoShip.addWaypoint(25, 23, 89, 0, 4, true);
		leftNearCargoShip.addWaypoint(23.8, 18, -91, 0, 3, false);
		leftNearCargoShip.addWaypoint(24, 22, -150, 0, 4, true);
		// leftNearCargoShip.addWaypoint(17, 20, -180, 4, 4, true);
		leftNearCargoShip.addWaypoint(2.5, 24.75, -180, 0, 10, false);

		leftNearCargoShip.addWaypoint(20, 20, -180, 8, 10, true);
		leftNearCargoShip.addWaypoint(23, 23, -91, 0, 6, true);
		leftNearCargoShip.addWaypoint(22, 18, -91, 0, 4, false);

		return leftNearCargoShip;
	}

	private FishyPath getFarCargoLeftSlow() {
		FishyPath leftNearCargoShip = new FishyPath(config, "HAB1L-CL3-Slow", DrivetrainSubsystem.WHEELBASE_FEET);
		// twoHatchRocketLeft.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		// twoHatchRocketLeft.addWaypointRelative(4.5, 0, 0, 3, 3, true);

		leftNearCargoShip.addWaypoint(new Waypoint(5.5, 17.354167, Math.PI, 0, 0));
		leftNearCargoShip.addWaypointRelative(4.5, 0, 0, 4, 4, true);
		leftNearCargoShip.addWaypoint(17, 17.5, -180, 4, 4, true);
		leftNearCargoShip.addWaypoint(24, 22, -100, 0, 4, true);
		// leftNearCargoShip.addWaypoint(25, 18, 89, 0, 4, false);

		// leftNearCargoShip.addWaypoint(25, 23, 89, 0, 4, true);
		leftNearCargoShip.addWaypoint(23.8, 18, -91, 0, 3, false);
		leftNearCargoShip.addWaypoint(24, 22, -150, 0, 4, true);
		// leftNearCargoShip.addWaypoint(17, 20, -180, 4, 4, true);
		leftNearCargoShip.addWaypoint(2.5, 24.75, -180, 0, 4, false);

		leftNearCargoShip.addWaypoint(20, 20, -180, 4, 4, true);
		leftNearCargoShip.addWaypoint(23, 23, -91, 0, 4, true);
		leftNearCargoShip.addWaypoint(22, 18, -91, 0, 4, false);

		return leftNearCargoShip;
	}

	private FishyPath getFarCargoLeftToLSL() {
		FishyPath leftCargoToLSL = new FishyPath(config, "CL3-LSL", DrivetrainSubsystem.WHEELBASE_FEET);
		leftCargoToLSL.addWaypoint(new Waypoint(25.5, 18, Math.toRadians(89), 0, 4));
		leftCargoToLSL.addWaypoint(25, 25, 89, 0, 4, true);
		leftCargoToLSL.addWaypoint(17, 20, 0, 4, 4, false);
		leftCargoToLSL.addWaypoint(2.5, 24.75, 0, 0, 4, false);
		return leftCargoToLSL;
	}

	private FishyPath getLSLToCargoMid() {
		FishyPath leftNearCargoShip = new FishyPath(config, "LSL-CL2", DrivetrainSubsystem.WHEELBASE_FEET);

		leftNearCargoShip.addWaypoint(new Waypoint(2.5, 24.75, 0, 0, 4));
		leftNearCargoShip.addWaypoint(20, 20, 0, 4, 4, true);
		leftNearCargoShip.addWaypoint(23, 23, 89, 0, 4, true);
		leftNearCargoShip.addWaypoint(23.5, 18, 89, 0, 4, true);
		return leftNearCargoShip;
	}

	private FishyPath getNearCargoRight() {
		FishyPath rightNearCargoShip = new FishyPath(config, "HAB1R-CR1", DrivetrainSubsystem.WHEELBASE_FEET);
		rightNearCargoShip.addWaypoint(5.5, 9.7, 0, 0, 0);
		rightNearCargoShip.addWaypointRelative(4.5, 0, 0, 6, 6, true);
		//rightNearCargoShip.addWaypoint(16.5, 9.25, 0, 6, 8.5, true, true);
		rightNearCargoShip.addWaypoint(21.7, 7, 0, 0, 8.5, true);
		return rightNearCargoShip;
	}

	private FishyPath getFarCargoRight() {
		FishyPath rightNearCargoShip = new FishyPath(config, "HAB1R-CR2", DrivetrainSubsystem.WHEELBASE_FEET);
		rightNearCargoShip.addWaypoint(5.5, 9.7, 0, 0, 0);
		rightNearCargoShip.addWaypointRelative(4.5, 0, 0, 6, 6, true);
		//rightNearCargoShip.addWaypoint(16.5, 9.25, 0, 6, 8.5, true, true);
		rightNearCargoShip.addWaypoint(24, 7, 0, 0, 8.5, true);
		return rightNearCargoShip;
	}

	private FishyPath getNearCargoLeftToLoadingStation() {
		FishyPath nearCargoLoadingStation = new FishyPath(config, "CL1-LSL", DrivetrainSubsystem.WHEELBASE_FEET);
		nearCargoLoadingStation.addWaypoint(21.7, 20, 0, 0, 0);
		nearCargoLoadingStation.addWaypoint(4.5, 24.75, 0, 0, 8.5, false);
		return nearCargoLoadingStation;
	}

	
	private FishyPath getNearCargoRightToLoadingStation() {
		FishyPath nearCargoLoadingStation = new FishyPath(config, "CR1-LSR", DrivetrainSubsystem.WHEELBASE_FEET);
		nearCargoLoadingStation.addWaypoint(21.7, 7, 0, 0, 0);
		nearCargoLoadingStation.addWaypoint(4.5, 2.25, 0, 0, 8.5, false);
		return nearCargoLoadingStation;
	}

	private FishyPath getFarCargoLeftToLoadingStation() {
		FishyPath farCargoLoadingStation = new FishyPath(config, "CL2-LSL", DrivetrainSubsystem.WHEELBASE_FEET);
		farCargoLoadingStation.addWaypoint(24, 20, 0, 0, 0);
		farCargoLoadingStation.addWaypoint(4.5, 24.75, 0, 0, 8.5, false);
		return farCargoLoadingStation;
	}
	
	private FishyPath getFarCargoRightToLoadingStation() {
		FishyPath nearCargoLoadingStation = new FishyPath(config, "CR2-LSR", DrivetrainSubsystem.WHEELBASE_FEET);
		nearCargoLoadingStation.addWaypoint(24, 7, 0, 0, 0);
		nearCargoLoadingStation.addWaypoint(4.5, 2.25, 0, 0, 8.5, false);
		return nearCargoLoadingStation;
	}

	private FishyPath getRocketToLoadingStationLeft() {
		FishyPath rocketLSLeft = new FishyPath(config, "ROCKLF-LSL", DrivetrainSubsystem.WHEELBASE_FEET);
		rocketLSLeft.addWaypoint(new Waypoint(20.0, 10.0, FishyMath.d2r(-29.5), 0, 0));
		rocketLSLeft.addWaypointRelative(4.5, -1.5, 29.5+25, 0, 10, true);
		rocketLSLeft.addWaypoint(3.0, 8.0, 0, 0.0, 12, false);
		return rocketLSLeft;
	}

	private FishyPath getNearRocketLeft() {
		FishyPath ianAssistLeft = new FishyPath(config, "NearRocketLeft", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssistLeft.addWaypoint(new Waypoint(5.5, 17.354167, 0, 0, 0));
		// ianAssistLeft.addWaypointRelative(2, 0, 0, 6, 6, true, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssistLeft.addWaypoint(14, 23.8, 30, 0, 6, false);
		return ianAssistLeft;
	}

	private FishyPath getNearRocketRight() {
		FishyPath ianAssist = new FishyPath(config, "NearRocketRight", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssist.addWaypoint(new Waypoint(5.5, 10.93750033, 0, 0, 0));
		// ianAssistLeft.addWaypointRelative(2, 0, 0, 6, 6, true, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		ianAssist.addWaypoint(14.0, 4.5, -30.0, 0.0, 6.0, false);
		return ianAssist;
	}

	private FishyPath getNearRocketToLoadingStation() {
		FishyPath ianAssist = new FishyPath(config, "NearRocketToLoadingStation", DrivetrainSubsystem.WHEELBASE_FEET);
		ianAssist.addWaypoint(4, 7, 0, 0, 6, false);
		ianAssist.addWaypoint(new Waypoint(14, 7, 0, 0, 0, false));
		return ianAssist;
	}

	private FishyPath getLoadingStationToNearCargoLeft() {
		FishyPath loadingStationNearCargo = new FishyPath(config, "LSL-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		//Robot.gyro.gyro.setAngleAdjustment(0);
		loadingStationNearCargo.addWaypoint(4.5, 24.75, 0, 7, 0);
		loadingStationNearCargo.addWaypoint(24, 19, 0, 0, 7, true);
		return loadingStationNearCargo;
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
		HAB1LxROCKLF.addWaypoint(25, 23.8, 0, 0, 11.7, true);
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
		twoHatchRocketLeft.addWaypointRelative(4.5, 0, 0, 3, 3, true); // go off HAB1 with max speed of 3 ft/s. waiting on specs from HW team for max speed off HAB2.
		twoHatchRocketLeft.addWaypoint(25, 24.3, 180, 0, 11.7, true);
		twoHatchRocketLeft.addWaypointRelative(-4.1, 1.0, -29.5, 0, 4.0, false);
		twoHatchRocketLeft.addWaypointRelative(3, -0.2, 29.5 + 30, 0, 4.0, true);
		twoHatchRocketLeft.addWaypoint(4, 24.8, 180, 0.0, 10.0, false);

		return twoHatchRocketLeft;
	}
}








		//  // Create a path with the name of"Example", this will generate a file named ExampleArc
		//  FishyPath exampleArc = new FishyPath(config, "FarRocketLeft", DrivetrainSubsystem.WHEELBASE_FEET);
		//  // Set the first point to the starating point, this be done with any of the addWaypoint methods
		//  // positive X is forward, positive Y is left, units ae in feet and degrees
		//  exampleArc.addWaypoint(HAB1L);
		//  // Add the next point that 4.5 ft forward, and doesn't turn, it also has a max speed of 4 FPS,
		//  // it will arrive at this location going 4 FPS
		//  exampleArc.addWaypointRelative(4.5, 0, 0, 3, 3);
		//  // Add the next point to be at global coordinates
		//  exampleArc.addWaypoint(25, 24, 0, 0, 11.5);
		// //  exampleArc.addWaypointRelative(-3.3, 0.675, -29.5, 0, 10.0);

		// FishyPath frontCargo = new FishyPath(config, "HABM-CF", DrivetrainSubsystem.WHEELBASE_FEET);
		// frontCargo.addWaypoint(5.5, 14.5, 0, 0, 0);
		// //frontCargo.addWaypointRelative(10.5, 0, 0, 0, 11.5);
		// frontCargo.addWaypointRelative(4, 0, 0, 3, 3);
    	// 	frontCargo.addWaypoint(16.25, 14.5, 0, 0, 6);


		// FishyPath nearCargoShip = new FishyPath(config, "HAB1L-CL1", DrivetrainSubsystem.WHEELBASE_FEET);
		// nearCargoShip.addWaypoint(HAB1L);
		// nearCargoShip.addWaypointRelative(4, 0, 0, 3, 3);
		// nearCargoShip.addWaypointRelative(6, 3, 25, 11.5, 11.5);
		// nearCargoShip.addWaypoint(21.7, 18, 89.99, 0, 11.5);

		// FishyPath frontToLoadingStation1 = new FishyPath(config, "CLF-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		// frontToLoadingStation1.addWaypoint(16, 14.5, 0, 0, 0);
		// frontToLoadingStation1.addWaypointRelative(-3, 3, 89.99, 2, 3);
		// frontToLoadingStation1.addWaypoint(16, 20.5, 179.98, 0, 3);

		// FishyPath frontToLoadingStation2 = new FishyPath(config, "CLF-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		// frontToLoadingStation2.addWaypoint(16, 20.5, 0, 0, 0);
		// frontToLoadingStation2.addWaypoint(1.5, 24.75, 0, 0, 11.5);

		// FishyPath nearRocketLeftToLoading1 = new FishyPath(config, "RLL-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		// nearRocketLeftToLoading1.addWaypoint(15.5, 24.25, 30, 0, 0);
		// nearRocketLeftToLoading1.addWaypoint(10, 18, 89.99, 0, 11.5);

		// FishyPath nearRocketLeftToLoading2 = new FishyPath(config, "RLL-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		// nearRocketLeftToLoading2.addWaypoint(10, 18, 89.99, 0, 0);
		// nearRocketLeftToLoading2.addWaypoint(1.5, 24.75, 0, 0, 11.5);

		// FishyPath loadingToNearRocketLeft1 = new FishyPath(config, "LSL-RLL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		// loadingToNearRocketLeft1.addWaypoint(1.5, 24.75, 0, 0, 0);
		// loadingToNearRocketLeft1.addWaypoint(10, 18, 89.99, 0, 11.5);

		// FishyPath loadingToNearRocketLeft2 = new FishyPath(config, "LSL-RLL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		// loadingToNearRocketLeft2.addWaypoint(10, 18, 89.99, 0, 0);
		// loadingToNearRocketLeft2.addWaypoint(15.5, 24.25, 30, 0, 11.5);

		// FishyPath loadingToMidCargoShip1 = new FishyPath(config, "LSL-CL2-1", DrivetrainSubsystem.WHEELBASE_FEET);
		// loadingToMidCargoShip1.addWaypoint(1.5, 24.75, 0, 0, 0);
		// loadingToMidCargoShip1.addWaypointRelative(15, -5, 15, 10, 11.5);
		// loadingToMidCargoShip1.addWaypoint(25, 24, 30, 0, 11.5);

		// FishyPath loadingToMidCargoShip2 = new FishyPath(config, "LSL-CL2-2", DrivetrainSubsystem.WHEELBASE_FEET);
		// loadingToMidCargoShip2.addWaypoint(25, 24, 30, 0, 0);
		// loadingToMidCargoShip2.addWaypoint(23.5, 18, 89.99, 0, 11.5);

		// FishyPath cargoShipLeft1ToLoading1 = new FishyPath(config, "CL1-LSL-1", DrivetrainSubsystem.WHEELBASE_FEET);
		// cargoShipLeft1ToLoading1.addWaypoint(21.75, 18, 89.99, 0, 0);
		// cargoShipLeft1ToLoading1.addWaypoint(25, 24, 30, 0, 11.5);

		// FishyPath cargoShipLeft1ToLoading2 = new FishyPath(config, "CL1-LSL-2", DrivetrainSubsystem.WHEELBASE_FEET);
		// cargoShipLeft1ToLoading2.addWaypoint(25, 24, 0, 0, 0);
		// cargoShipLeft1ToLoading2.addWaypointRelative(-8.5, -4.25, -15, 10, 11.5);
		// cargoShipLeft1ToLoading2.addWaypoint(4.5, 24.75, 0, 0, 11.5);

		// FishyPath straightPath = new FishyPath(config, "straight", DrivetrainSubsystem.WHEELBASE_FEET);
		// straightPath.addWaypoint(10, 20, 0, 0, 0);
		// straightPath.addWaypoint(24, 20, 0, 0, 11.5);

		// FishyPath revPath = new FishyPath(config, "rev", DrivetrainSubsystem.WHEELBASE_FEET);
		// revPath.addWaypoint(24, 20, 0, 0, 0);
		// revPath.addWaypoint(10, 20, 0, 0, 11.5);

		// FishyPath anglePath = new FishyPath(config, "angle", DrivetrainSubsystem.WHEELBASE_FEET);
		// anglePath.addWaypoint(24, 24, 45, 0, 0);
		// anglePath.addWaypoint(15, 15, 45, 0, 11.5);