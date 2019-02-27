/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;

public class RunCargoIntake extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RunCargoIntake(double power) {
    addParallel(new ChangeIntakeState(CargoIntakeState.OUT));
    addSequential(new SetIntakeRollers(true, power));
    addSequential(new UpdateIntakeStateCommand());
  }
}
