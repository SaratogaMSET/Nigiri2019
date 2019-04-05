/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Robot;
import frc.robot.subsystems.CargoDeploySubsystem.CargoDeployMotorState;
import frc.robot.subsystems.CargoDeploySubsystem.CargoGamePiece;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeMotorState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchDeployState;
import frc.robot.subsystems.HatchSubsystem.HatchGamePiece;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

/**
 * Add your docs here.
 */
public class RobotState {

    public static LiftPositions liftPosition;
    public static LiftPositions lastLiftTarget;
    public static CargoIntakePositionState cargoIntakeState;
    public static CargoIntakeMotorState intakeMotorState;
    public static HatchPositionState hatchPositionState;
    public static HatchDeployState hatchDeployState;
    public static HatchGamePiece hatchGamePiece;
    public static CargoDeployMotorState cargoDeployState;
    public static CargoGamePiece cargoGamePiece;

    public static boolean isRunningLiftCommand = false;
    
    public RobotState() {
        liftPosition = LiftPositions.LOW;
        hatchPositionState = HatchPositionState.HATCH_OUT;
        hatchDeployState = HatchDeployState.HOLD;
        hatchGamePiece = HatchGamePiece.NO_HATCH;
        intakeMotorState = CargoIntakeMotorState.NONE;
        cargoIntakeState = CargoIntakePositionState.IN;
        cargoDeployState = CargoDeployMotorState.NONE;
        cargoGamePiece = CargoGamePiece.NO_CARGO;
        lastLiftTarget = liftPosition;
    }

    public static boolean canRunLift() {
        if (cargoIntakeState == CargoIntakePositionState.MID || cargoIntakeState == CargoIntakePositionState.OUT || 
        (cargoIntakeState == CargoIntakePositionState.MOVING && Robot.cargoIntake.getMidStateSolState())) {
            return true;
        }
        return false;
    }

    public static boolean canBringIntakeIn() {
        if(liftPosition == LiftPositions.LOW || liftPosition == LiftPositions.CARGO_LOADING_STATION || liftPosition == LiftPositions.CARGO_SHIP) {
            return true;
        } 
        return false;
    }

    public static boolean runIntakesWhileLifting() {
        return (Robot.lift.getRawEncoder() < Robot.lift.getLiftPositionEncoders(LiftPositions.CARGO_ROCKET_LEVEL_ONE))? true : false;
    }

    public static boolean isLiftCargoState(LiftPositions current) {
        if(current != LiftPositions.HATCH_MID && current != LiftPositions.HATCH_HIGH) {
            return true;
        } else if(current == LiftPositions.LOW && hatchPositionState == HatchPositionState.HATCH_IN) {
            return true;
        }
        return false;
    }
}
