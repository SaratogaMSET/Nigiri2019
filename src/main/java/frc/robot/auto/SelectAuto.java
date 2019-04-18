/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.semiauto.AutoIntakeHatch;
import frc.robot.commands.test.TestDTMaxVA;
import frc.robot.commands.test.TestTalonVelocity;
import frc.robot.commands.test.TuneMotionProfile;
import frc.robot.subsystems.AutoSelector.Side;
import frc.robot.subsystems.AutoSelector.Control;

public class SelectAuto extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SelectAuto() {
    
    Control control = Robot.autoSelector.getControl();

    if(control == Control.TELEOP) {
      addSequential(new AutoIntakeHatch());
    } else {
      // addParallel(new AutoIntakeHatch());
      addSequential(Robot.autoCommand);
    }
    addSequential(new Command() {
      @Override
      protected void initialize() {
          Robot.switchAutoToTeleop();
      }
      @Override
      protected boolean isFinished() {
        return Robot.autoControl == false;
      }
    });
  }

  public static void chooseAuto(int autoNumber, Side side) {
    switch(autoNumber) {
      case 1:
        if(side == Side.LEFT) {
          Robot.autoCommand = new DoubleRocket(true);
          SmartDashboard.putString("current Auto", "double rocket Left");
        } else {
          Robot.autoCommand = new DoubleRocket(false);
          SmartDashboard.putString("current Auto", "double rocket Right");
        }
        break;
      case 2:
        if(side == Side.LEFT) {
          Robot.autoCommand = new IanAssistedDrive(false, true);
          SmartDashboard.putString("current Auto", "back rocket no deploy left slow");
        } else {
          Robot.autoCommand = new IanAssistedDrive(true, true);
          SmartDashboard.putString("current Auto", "back rocket no deploy right slow");
        }
        break;
      case 3:
        Robot.autoCommand = new DoubleCargoShip(false);
        SmartDashboard.putString("current Auto", "double cargo");
        break;
      case 4:
        Robot.autoCommand = new DoubleCargoShip(true);
        SmartDashboard.putString("current Auto", "double cargo slow");
        break;
      case 5:
        if(side == Side.LEFT) {
          Robot.autoCommand = new TestDTMaxVA(20);
          SmartDashboard.putString("current Auto", "TestDTMaxVA");
        } else {
          Robot.autoCommand = new TestTalonVelocity(20);
          SmartDashboard.putString("current Auto", "TestTalonVelocity");
        }
        break;
      case 6:
        if(side == Side.LEFT) {
          Robot.autoCommand = new TuneMotionProfile("StraightFastLong");
          SmartDashboard.putString("current Auto", "StraightFastLong");
        } else {
          Robot.autoCommand = new TuneMotionProfile("StraightFastLongReverse");
          SmartDashboard.putString("current Auto", "StraightFastLongReverse");
        }
        break;
      case 7:
        if(side == Side.LEFT) {
          Robot.autoCommand = new TuneMotionProfile("TurnScaling");
          SmartDashboard.putString("current Auto", "TurnScaling");
        } else {
          Robot.autoCommand = new TuneMotionProfile("TurnScalingReverse");
          SmartDashboard.putString("current Auto", "TurnScalingReverse");
        }
        break;
      case 8:
        if(side == Side.LEFT) {
          Robot.autoCommand = new TuneMotionProfile("StrightSlowLong");
          SmartDashboard.putString("current Auto", "StrightSlowLong");
        } else {
          Robot.autoCommand = new TuneMotionProfile("StraightSlowLongReverse");
          SmartDashboard.putString("current Auto", "StraightSlowLongReverse");
        }
        break;
      case 9:
        if(side == Side.LEFT) {
          Robot.autoCommand = new TuneMotionProfile("StraightFastShort");
          SmartDashboard.putString("current Auto", "StraightFastShort");
        } else {
          Robot.autoCommand = new TuneMotionProfile("StraightFastShortReverse");
          SmartDashboard.putString("current Auto", "StraightFastShortReverse");
        }
        break;
      case 10:
        if(side == Side.LEFT) {
          Robot.autoCommand = new IanAssistedDrive(false, false);
          SmartDashboard.putString("current Auto", "back rocket no deploy left");
        } else {
          Robot.autoCommand = new IanAssistedDrive(true, false);
          SmartDashboard.putString("current Auto", "back rocket no deploy right");
        }
        break;
    }
  }
}
