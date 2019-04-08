/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.CloseHatchCommand;
import frc.robot.commands.DeployHatchCommand;
import frc.robot.commands.MoveLiftCommand;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;

public class AutoStateMachine extends Command {

  Action[] actions;
  int index;
  boolean isFinished;
  public AutoStateMachine(String pathName) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    if(pathName == "DoubleHatchCargoShip") {
      setUpDoubleCargo();
    } else if(pathName == "test") {
      test();
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    isFinished = false;
    index = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    try {
      if(actions[index].triggered(Robot.drive.getLeftEncoderDistance())) {
        actions[index].getCommand().start();
        index++;
      }
    } catch (ArrayIndexOutOfBoundsException e) {
      isFinished = true;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !Robot.autoControl || index >= actions.length || isFinished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  public void setUpDoubleCargo() {
    Action[] actions = {
      new Action(new MoveLiftCommand(LiftPositions.AUTO_CARGO_SHIP_HATCH, 0.6), -22.5, true), // move lift up
      new Action(new DeployHatchCommand(), -22, true), // deploy first hatch
      new Action(new MoveLiftCommand(LiftPositions.LOW, 0.6), -22, false), // move lift down
      new Action(new CloseHatchCommand(), -15, true), // close hatch mech
      new Action(new MoveLiftCommand(LiftPositions.AUTO_CARGO_SHIP_HATCH, 0.6), -23, true), // move lift up
      new Action(new DeployHatchCommand(), -22, true) // deploy hatch
    };

    this.actions = actions;
  }

  public void test() {
    Action[] actions = {
      new Action(new DeployHatchCommand(), 3, true),
      new Action(new CloseHatchCommand(), 6, true),
      new Action(new DeployHatchCommand(), 3, false),
      new Action(new CloseHatchCommand(), 0, false),
      new Action(new DeployHatchCommand(), -3, false),
      new Action(new CloseHatchCommand(), 0, true)
    };
  }
  public static class Action {
    private Command command;
    private double encoderTick;
    private boolean forward;
    private final double TOLERANCE = 0.5;

    public Action(Command command, double encoderTick, boolean forward) {
      this.command = command;
      this.encoderTick = encoderTick;
      this.forward = forward;
    }

    public boolean triggered(double current) {
      if(Math.abs(current - encoderTick) < TOLERANCE) {
        if(isGoingForward() == forward) {
          return true;
        }
      }
      return false;
    }

    public boolean isGoingForward() {
      return Robot.drive.getLeftEncoderVelocity() > 0 ? true : false;
    }

    public Command getCommand() {
      return command;
    }
  }
}
