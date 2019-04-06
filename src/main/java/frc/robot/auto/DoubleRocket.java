/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.commands.semiauto.HatchMid;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.commands.EncoderTrigger;

public class DoubleRocket extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DoubleRocket(boolean isLeft) {
    addParallel(new EncoderTrigger(-21.27, new HatchMid()));
    addSequential(new MotionProfileCommand("HAB1LxROCKLFxLOADLxROCKLF", 180.0));
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
