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
import frc.robot.commands.WaitUntilEncoderCommand;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.commands.DeployHatchCommand;


public class HAB1LxCLFxLOADLxCL1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public HAB1LxCLFxLOADLxCL1() {//215
    addParallel(new ChangeIntakeState(CargoIntakePositionState.MID));
    addParallel(new WaitUntilEncoderCommand(2, new MoveHatchCommand(HatchPositionState.HATCH_OUT), 10));
    addParallel(new WaitUntilEncoderCommand(6, new DeployHatchCommand(), 20));
    // addSequential(new MotionProfileCommand("Straight", false, false));
    // addSequential(new MotionProfileCommand("3-point2", true)); //.002

    // addSequential(new MotionProfileCommand("TurnToLoadingStation", false));
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
  }
}
