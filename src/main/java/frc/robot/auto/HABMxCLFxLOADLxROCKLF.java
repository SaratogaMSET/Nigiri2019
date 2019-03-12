/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.MoveHatchCommand;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;

public class HABMxCLFxLOADLxROCKLF extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HABMxCLFxLOADLxROCKLF() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new ChangeIntakeState(CargoIntakePositionState.MID));
    addSequential(new MoveHatchCommand(HatchPositionState.HATCH_OUT));
    addSequential(new MotionProfileCommand("HAB1L-CL1", false));
    /*addSequential(new MotionProfileCommand("CLF-LSL-1", true));
    addSequential(new MotionProfileCommand("CLF-LSL-2", false));*/
    //addSequential(new MotionProfileCommand("LSL-RLL-1", true));
    //addSequential(new MotionProfileCommand("LSL-RLL-2", false));
  }
}
