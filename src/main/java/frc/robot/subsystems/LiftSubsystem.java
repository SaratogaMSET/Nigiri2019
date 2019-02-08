/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum LiftPositions {
    low,
    cargoRocketLevelOne,
    cargoRocketLevelTwo,
    cargoRocketLevelThree,
    cargoLoadingStation,
    hatchMid,
    hatchHigh,
    climbHabTwo,
    climbHabThree
  }

  public static class LiftEncoderConstants {
    public static final int INTAKE = 0;
    public static final int CARGO_ROCKET_LEVEL_ONE = 0;
    public static final int CARGO_ROCKET_LEVEL_TWO = 0;
    public static final int CARGO_ROCKET_LEVEL_THREE = 0;
    public static final int CARGO_LOADING_STATION = 0;
    public static final int HATCH_MID = 0;
    public static final int HATCH_HIGH = 0;
    public static final int CLIMB_HAB_TWO = 2728 + 440;
    public static final int CLIMB_HAB_THREE = 12500 + 440;
    public static final double LIFT_TICKS_PER_JACK_TICK = 1.2/1.75; //for every tick of jack go this much lift
  }

  public static class LiftPidConstants {
    // Use feedBACK only for the downwards lift pushing for the climb.
    public static final double CLIMB_kF = 0.0;
    public static final double CLIMB_kP = 0.06;
    public static final double CLIMB_kI = 0.0;
    public static final double CLIMB_kD = 0.0;


  }

  private TalonSRX motor1;
  private TalonSRX motor2;
  private TalonSRX motor3;
  private DigitalInput bottomHal;
  private DigitalInput topHal;
  private LiftPositions currentPosition;

  public LiftSubsystem() {
    motor1 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_1_PORT);
    motor2 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_2_PORT);
    motor3 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_3_PORT);

    // bottomHal = new DigitalInput(RobotMap.Lift.BOTTOM_HAL_EFFECT);
    // topHal = new DigitalInput(RobotMap.Lift.TOP_HAL_EFFECT);

    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor3.set(ControlMode.Follower, motor1.getDeviceID());

    motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    currentPosition = LiftPositions.low;
    
  }

  public void setManualLift(double power) {
    motor1.set(ControlMode.PercentOutput, power);
  }

  public void setLiftMotor(int id, double power) {
    if(id == 1) {
      motor1.set(ControlMode.PercentOutput, power);
    } else if(id == 2) {
      motor2.set(ControlMode.PercentOutput, power);
    } else if(id == 3) {
      motor3.set(ControlMode.PercentOutput, power);
    }
  }

  public int getLiftMotorID(int id) {
    if(id == 1) {
      return motor1.getDeviceID();
    } else if(id == 2) {
      return motor2.getDeviceID();
    } else if(id == 3) {
      return motor3.getDeviceID();
    }
    return 0;
  }

  public void setLiftMPHang(){
    motor1.config_kF(0, LiftPidConstants.CLIMB_kF, Robot.timeoutMs);
    motor1.config_kP(0, LiftPidConstants.CLIMB_kP, Robot.timeoutMs);
    motor1.config_kI(0, LiftPidConstants.CLIMB_kI, Robot.timeoutMs);
    motor1.config_kD(0, LiftPidConstants.CLIMB_kD, Robot.timeoutMs);
    // motor1.configMotionCruiseVelocity(LiftPidConstants.HANG_VEL, Robot.timeoutMs);
    // motor1.configMotionAcceleration(LiftPidConstants.HANG_ACCEL, Robot.timeoutMs);

  }

  public void motionMagicLift(int pos) {
    motor1.set(ControlMode.MotionMagic, pos);
  }
  public void pidLift(int pos){
    motor1.set(ControlMode.Position, pos);
  }

  public void resetEncoder() {
    motor1.setSelectedSensorPosition(0);
  }

  public int getRawEncoder() {
    return motor1.getSelectedSensorPosition();
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
      case climbHabTwo:
        motionMagicLift(LiftEncoderConstants.CLIMB_HAB_TWO);
        break;
      case climbHabThree:
        motionMagicLift(LiftEncoderConstants.CLIMB_HAB_THREE);
        break;    
    }
  }

  public void setPosition(LiftPositions pos) {
    currentPosition = pos;
  }

  public LiftPositions getLiftPosition() {
    return currentPosition;
  }

  // public boolean getTopHal() {
  //   return topHal.get();
  // }

  // public boolean getBottomHal() {
  //   return bottomHal.get();
  // }

  public void setFollowers() {
    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor3.set(ControlMode.Follower, motor1.getDeviceID());
  }

  public void setPercentOutput() {
    motor2.set(ControlMode.PercentOutput, 0);
    motor3.set(ControlMode.PercentOutput, 0);
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
