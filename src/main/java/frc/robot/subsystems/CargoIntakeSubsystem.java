/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class CargoIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum CargoIntakeState {
    OUT,
    MID,
    IN,
    INTAKE,
    EXTAKE,
    NONE
  }
  
  private TalonSRX leftIntake, rightIntake, frontIntake;
  private Solenoid intakeSol;
  public Solenoid intakeMidSol;
  private boolean isOut;
  private DigitalInput outHal;
  private DigitalInput inHal;

  public CargoIntakeSubsystem(){

    leftIntake = new TalonSRX(RobotMap.CargoIntake.LEFT_INTAKE);
    rightIntake = new TalonSRX(RobotMap.CargoIntake.RIGHT_INTAKE);
    frontIntake = new TalonSRX(RobotMap.CargoIntake.BACK_INTAKE);

    intakeSol = new Solenoid(4, RobotMap.CargoIntake.INTAKE_SOL[0]);
    intakeMidSol = new Solenoid(4, RobotMap.CargoIntake.INTAKE_SOL[1]);
    
    outHal = new DigitalInput(RobotMap.CargoIntake.INTAKE_DOWN_HAL);
    inHal = new DigitalInput(RobotMap.CargoIntake.INTAKE_UP_HAL);

    intakeSol.set(false);
    intakeMidSol.set(false);

    leftIntake.configNominalOutputForward(0, Robot.timeoutMs);
    leftIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    leftIntake.configPeakOutputForward(1, Robot.timeoutMs);
    leftIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    rightIntake.configNominalOutputForward(0, Robot.timeoutMs);
    rightIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    rightIntake.configPeakOutputForward(1, Robot.timeoutMs);
    rightIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    frontIntake.configNominalOutputForward(0, Robot.timeoutMs);
    frontIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    frontIntake.configPeakOutputForward(1, Robot.timeoutMs);
    frontIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

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
      rightIntake.set(ControlMode.PercentOutput, -power);
      frontIntake.set(ControlMode.PercentOutput, power);
    } else {
      leftIntake.set(ControlMode.PercentOutput, -power);
      rightIntake.set(ControlMode.PercentOutput, power);
      frontIntake.set(ControlMode.PercentOutput, -power);
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

  public CargoIntakeState getIntakeState() {
    if(intakeSol.get()) {
      return CargoIntakeState.OUT;
    } else if(intakeSol.get() == false && intakeMidSol.get()) {
      return CargoIntakeState.IN;
    } else {
      return CargoIntakeState.MID;
    }
  }

  public boolean getMidStateSolState() {
    return intakeMidSol.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
