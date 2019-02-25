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
    public static CargoIntakeState intakeState;
    public static HatchPositions hatchState;

    public RobotState() {
        liftPosition = LiftPositions.LOW;
        intakeState = CargoIntakeState.MID;
        hatchState = HatchPositions.hatchOut;
    }
    public boolean canRunLift() {
        if (intakeState == CargoIntakeState.MID || intakeState == CargoIntakeState.OUT) {
            return true;
        }
        return false;
    }
}
