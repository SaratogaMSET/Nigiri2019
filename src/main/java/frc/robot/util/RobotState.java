/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;
import frc.robot.subsystems.HatchSubsystem.HatchPositions;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

/**
 * Add your docs here.
 */
public class RobotState {
    public static LiftPositions liftPosition;
    public static CargoIntakeState cargoIntakeState;
    public static HatchPositions hatchState;
    public static CargoIntakeState intakeState;

    public RobotState() {
        liftPosition = LiftPositions.LOW;
        cargoIntakeState = CargoIntakeState.MID;
        hatchState = HatchPositions.hatchOut;
        intakeState = CargoIntakeState.NONE;
    }
    public boolean canRunLift() {
        if (cargoIntakeState == CargoIntakeState.MID || cargoIntakeState == CargoIntakeState.OUT) {
            return true;
        }
        return false;
    }

    public boolean canBringIntakeIn() {
        if(liftPosition == LiftPositions.LOW) {
            return true;
        }
        return false;
    }
}
