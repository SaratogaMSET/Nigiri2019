/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum LiftPositions {
    low,
    cargoRocketLevelOne,
    cargoRocketLevelTwo,
    cargoRocketLevelThree,
    cargoLoadingStation,
    hatchMid,
    hatchHigh
  }

  public static class LiftEncoderConstants {
    public static final int INTAKE = 0;
    public static final int CARGO_ROCKET_LEVEL_ONE = 0;
    public static final int CARGO_ROCKET_LEVEL_TWO = 0;
    public static final int CARGO_ROCKET_LEVEL_THREE = 0;
    public static final int CARGO_LOADING_STATION = 0;
    public static final int HATCH_MID = 0;
    public static final int HATCH_HIGH = 0;
  }
  private TalonSRX motor1;
  private TalonSRX motor2;
  private TalonSRX motor3;
  private LiftPositions currentPosition;

  public LiftSubsystem() {
    motor1 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_1_PORT);
    motor2 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_2_PORT);
    motor3 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_3_PORT);

    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor3.set(ControlMode.Follower, motor1.getDeviceID());

    currentPosition = LiftPositions.low;
  }

  public void setRawLift(double power) {
    motor1.set(ControlMode.PercentOutput, power);
  }

  public void motionMagicLift(int pos) {
    motor1.set(ControlMode.MotionMagic, pos);
  }

  public void moveLiftToPos(LiftPositions pos) {
    switch(pos) {
      case low:
        motionMagicLift(LiftEncoderConstants.INTAKE);
        break;
      case cargoRocketLevelOne:
        motionMagicLift(LiftEncoderConstants.CARGO_ROCKET_LEVEL_ONE);
        break;
      case cargoRocketLevelTwo:
        motionMagicLift(LiftEncoderConstants.CARGO_ROCKET_LEVEL_TWO);
        break;
      case cargoRocketLevelThree:
        motionMagicLift(LiftEncoderConstants.CARGO_ROCKET_LEVEL_THREE);
        break;
      case cargoLoadingStation:
        motionMagicLift(LiftEncoderConstants.CARGO_LOADING_STATION);
        break;
      case hatchMid:
        motionMagicLift(LiftEncoderConstants.HATCH_MID);
        break;
      case hatchHigh:
        motionMagicLift(LiftEncoderConstants.HATCH_HIGH);
        break;
    }
  }

  public void setPosition(LiftPositions pos) {
    currentPosition = pos;
  }

  public LiftPositions getLiftPosition() {
    return currentPosition;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void diagnosticShuffleboard() {
    
  }

  @Override
  public void essentialShuffleboard() {
    
  }
}
