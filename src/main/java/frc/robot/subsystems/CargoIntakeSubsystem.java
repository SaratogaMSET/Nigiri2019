/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.util.Logging;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class CargoIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum CargoIntakePositionState {
    OUT,
    MID,
    IN,
    MOVING
  }

  public static enum CargoIntakeMotorState {
    INTAKE,
    EXTAKE,
    TOP_BAR_ONLY,
    NONE,
    FAIL_STATE
  }
  
  private TalonSRX leftIntake, rightIntake, frontIntake;
  private Solenoid intakeSol;
  public Solenoid intakeMidSol;
  private boolean isOut;
  private DigitalInput outHal;
  private DigitalInput inHal;

  private int invalidStateCount;

  public CargoIntakeSubsystem(){

    leftIntake = new TalonSRX(RobotMap.CargoIntake.LEFT_INTAKE);
    rightIntake = new TalonSRX(RobotMap.CargoIntake.RIGHT_INTAKE);
    frontIntake = new TalonSRX(RobotMap.CargoIntake.BACK_INTAKE);

    intakeSol = new Solenoid(RobotMap.PCM, RobotMap.CargoIntake.INTAKE_SOL[0]);
    intakeMidSol = new Solenoid(RobotMap.PCM, RobotMap.CargoIntake.INTAKE_SOL[1]);
    
    outHal = new DigitalInput(RobotMap.CargoIntake.INTAKE_DOWN_HAL);
    inHal = new DigitalInput(RobotMap.CargoIntake.INTAKE_UP_HAL);

    invalidStateCount = 0;

    intakeSol.set(false);
    intakeMidSol.set(false);

    leftIntake.configNominalOutputForward(0, Robot.timeoutMs);
    leftIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    leftIntake.configPeakOutputForward(1, Robot.timeoutMs);
    leftIntake.configPeakOutputReverse(-1, Robot.timeoutMs);
    leftIntake.setInverted(true);

    rightIntake.configNominalOutputForward(0, Robot.timeoutMs);
    rightIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    rightIntake.configPeakOutputForward(1, Robot.timeoutMs);
    rightIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    frontIntake.configNominalOutputForward(0, Robot.timeoutMs);
    frontIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    frontIntake.configPeakOutputForward(1, Robot.timeoutMs);
    frontIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    frontIntake.configPeakCurrentDuration(3, Robot.timeoutMs);
    frontIntake.configPeakCurrentLimit(40, Robot.timeoutMs);
    frontIntake.configContinuousCurrentLimit(40, Robot.timeoutMs);
    frontIntake.enableCurrentLimit(true);
    // leftIntake.configPeakCurrentLimit(20, Robot.timeoutMs);
    // rightIntake.configPeakCurrentLimit(20, Robot.timeoutMs);
    // frontIntake.configPeakCurrentLimit(20, Robot.timeoutMs);

    isOut = false;
  }

  public void runMotor(int motor, double power){
    switch(motor){
      case 1:
        leftIntake.set(ControlMode.PercentOutput, power);
        break;
      case 2:
        rightIntake.set(ControlMode.PercentOutput, -power);
        break;
      case 3:
        frontIntake.set(ControlMode.PercentOutput, power);
        break;
    }
  }

  public void testAll (double power){
    leftIntake.set(ControlMode.PercentOutput, power);
    rightIntake.set(ControlMode.PercentOutput, -power);
    frontIntake.set(ControlMode.PercentOutput, power);

  }

  public void runIntake(boolean in, double power) {
    if(in) {
      leftIntake.set(ControlMode.PercentOutput, power);
      rightIntake.set(ControlMode.PercentOutput, power);
      frontIntake.set(ControlMode.PercentOutput, power);
    } else {
      leftIntake.set(ControlMode.PercentOutput, -power);
      rightIntake.set(ControlMode.PercentOutput, -power);
      frontIntake.set(ControlMode.PercentOutput, -power);
    }
  }

  public void runIntake(boolean in, double topPower, double sidePower) {
    if(in) {
      leftIntake.set(ControlMode.PercentOutput, sidePower);
      rightIntake.set(ControlMode.PercentOutput, sidePower);
      frontIntake.set(ControlMode.PercentOutput, topPower);
    } else {
      leftIntake.set(ControlMode.PercentOutput, -sidePower);
      rightIntake.set(ControlMode.PercentOutput, -sidePower);
      frontIntake.set(ControlMode.PercentOutput, -topPower);
    }
  }

  public void switchSol(boolean state){
    intakeSol.set(state);
    SmartDashboard.putBoolean("is Out", state);
  }

  public void setMidStateSol(boolean state) {
    intakeMidSol.set(state);
  }

  public boolean solOut(){
    return isOut;
  }

  public boolean getIntakeSolState() {
    return intakeSol.get();
  }

  public boolean getOutHal() {
    return !outHal.get();
  }

  public boolean getInHal() {
    return !inHal.get();
  }

  public CargoIntakePositionState updateIntakeState() {
    if(getInHal()) {
      return CargoIntakePositionState.IN;
    }
    if(intakeSol.get()) {
      if(getOutHal()) {
        return CargoIntakePositionState.OUT;
      } else {
        return CargoIntakePositionState.MOVING;
      }
    } else if(!intakeSol.get() && !intakeMidSol.get()) {
      if(getInHal()) {
        return CargoIntakePositionState.IN;
      } else {
        return CargoIntakePositionState.MOVING;
      }
    } else {
      return CargoIntakePositionState.MID;
    }
  }

  public CargoIntakeMotorState updateIntakeRollerState() {
    double leftPower = leftIntake.getMotorOutputPercent();
    double rightPower = rightIntake.getMotorOutputPercent();
    double topPower = frontIntake.getMotorOutputPercent();

    if(topPower > 0 && rightPower == 0 && leftPower == 0) {
      return CargoIntakeMotorState.TOP_BAR_ONLY;
    } else if(topPower > 0 && rightPower > 0 && leftPower > 0) {
      return CargoIntakeMotorState.INTAKE;
    } else if(topPower == 0 && rightPower == 0 && leftPower == 0) {
      return CargoIntakeMotorState.NONE;
    } else {
      return CargoIntakeMotorState.EXTAKE;
    }
  }

  public boolean getMidStateSolState() {
    return intakeMidSol.get();
  }

  public void smartdashboard() {
    SmartDashboard.putNumber("Left Intake Current", leftIntake.getOutputCurrent());
    SmartDashboard.putNumber("Right Intake Current", rightIntake.getOutputCurrent());
    SmartDashboard.putNumber("Front Intake Current", frontIntake.getOutputCurrent());
  }

  public void checkIntakeState() {
    CargoIntakePositionState posState = RobotState.cargoIntakeState;
    CargoIntakeMotorState motorState = RobotState.intakeMotorState;

    if(posState == CargoIntakePositionState.OUT) {
      if(motorState == CargoIntakeMotorState.TOP_BAR_ONLY) {
        invalidStateCount++;
      } else if(motorState == CargoIntakeMotorState.NONE) {
        invalidStateCount++;
      } else {
        invalidStateCount = 0;
      }
    } else if(posState == CargoIntakePositionState.MID) {
      if(motorState == CargoIntakeMotorState.INTAKE) {
        invalidStateCount++;
      } else if(motorState == CargoIntakeMotorState.EXTAKE) {
        invalidStateCount++;
      } else {
        invalidStateCount = 0;
      }
    } else if(posState == CargoIntakePositionState.IN) {
      if(motorState == CargoIntakeMotorState.INTAKE) {
        invalidStateCount++;
      } else if(motorState == CargoIntakeMotorState.EXTAKE) {
        invalidStateCount++;
      } else if(motorState == CargoIntakeMotorState.TOP_BAR_ONLY) {
        invalidStateCount++;
      } else {
        invalidStateCount = 0;
      }
    } 
    SmartDashboard.putNumber("Invalid State", invalidStateCount);
    // fixState(posState, motorState, invalidStateCount);
    updateIntakeRollerState();
    updateIntakeState();
  }

  private void fixState(CargoIntakePositionState pos, CargoIntakeMotorState motor, int count) {
    if (count >= 10) {
      if(Robot.isLogging) {
        String string = String.format("%.4f, FixState For CargoIntake", Robot.time.get());
        Logging.print(string);
      }
      switch(pos) {
        case OUT:
          if((Robot.oi.gamePad.getLeftButton() && RobotState.liftPosition == LiftPositions.LOW)) {
            new SetIntakeRollers(true, 1).start();
          } else if(Robot.lift.isMoving()) {
            new SetIntakeRollers(true, 0).start();
            new ChangeIntakeState(CargoIntakePositionState.MID).start();
          }
          break;
        case MID:
          new SetIntakeRollers(true, 0).start();
          break;
        case IN:
          new SetIntakeRollers(true, 0).start();
          break;
      }
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void essentialShuffleboard() {
    
  }

  @Override
  public void diagnosticShuffleboard() {
    
  }

  @Override
  public void stopAll() {
    // TODO: Change this if u set the intakes to followers
    leftIntake.set(ControlMode.PercentOutput, 0);
    rightIntake.set(ControlMode.PercentOutput, 0);
    frontIntake.set(ControlMode.PercentOutput, 0);
  }
}
