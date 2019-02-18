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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum LiftPositions {
    LOW,
    CARGO_SHIP,
    CARGO_ROCKET_LEVEL_ONE,
    CARGO_ROCKET_LEVEL_TWO,
    CARGO_ROCKET_LEVEL_THREE,
    CARGO_LOADING_STATION,
    HATCH_MID,
    HATCH_HIGH,
    CLIMB_HAB_TWO,
    CLIMB_HAB_THREE
  }

  public static class LiftTargetEncoderTicks {
    public static final int CLIMB_HAB_TWO = 2728 + 440;
    public static final int CLIMB_HAB_THREE = 12500 + 440;
    public static final double LIFT_TICKS_PER_JACK_TICK = 1.2/1.75; //for every tick of jack go this much lift
    public static final double DISTANCE_PER_PULSE = 1.75 * 2 * Math.PI / 4096.0;
    public static final int TOLERANCE = 100;
  }

  public static class LiftTargetInches {
    // in. from 0-state
    public static final double INTAKE = 0;
    public static final double CARGO_SHIP = 30;
    public static final double CARGO_ROCKET_LEVEL_ONE = 18.5;
    public static final double CARGO_ROCKET_LEVEL_TWO = 46.04;
    public static final double CARGO_ROCKET_LEVEL_THREE = 83.04;
    public static final double CARGO_LOADING_STATION = 0;
    public static final double HATCH_MID = 46.6;
    public static final double HATCH_HIGH = 74.6;
  }

  public static class PIDConstants {
    public static final double k_f = 0.2925714;
    public static double k_p = 0.0;
    public static double k_i = 0.0;
    public static double k_d = 0.0;
    public static final int MAX_ACCELERATION = 10000; //measured 40000-70000
    public static final int MAX_VELOCITY = 3500; // measured 4500

    public static final double CLIMB_kF = 0.0;
    public static final double CLIMB_kP = 0.06;
    public static final double CLIMB_kI = 0.0;
    public static final double CLIMB_kD = 0.0;
  }

  public TalonSRX master;
  private TalonSRX motor2;
  private TalonSRX motor3;

  public DigitalInput bottomHal;
  public DigitalInput topHal;
  public LiftPositions currentPosition;
  private boolean isMoving;

  public LiftSubsystem() {
    master = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_1_PORT);
    motor2 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_2_PORT);
    motor3 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_3_PORT);

    // bottomHal = new DigitalInput(RobotMap.Lift.BOTTOM_HAL_EFFECT);
    // topHal = new DigitalInput(RobotMap.Lift.TOP_HAL_EFFECT);

    master.configNominalOutputForward(0, Robot.timeoutMs);
    master.configNominalOutputReverse(0, Robot.timeoutMs);
    master.configPeakOutputForward(1, Robot.timeoutMs);
    master.configPeakOutputReverse(-1, Robot.timeoutMs);

    motor2.configNominalOutputForward(0, Robot.timeoutMs);
    motor2.configNominalOutputReverse(0, Robot.timeoutMs);
    motor2.configPeakOutputForward(1, Robot.timeoutMs);
    motor2.configPeakOutputReverse(-1, Robot.timeoutMs);

    motor3.configNominalOutputForward(0, Robot.timeoutMs);
    motor3.configNominalOutputReverse(0, Robot.timeoutMs);
    motor3.configPeakOutputForward(1, Robot.timeoutMs);
    motor3.configPeakOutputReverse(-1, Robot.timeoutMs);

    motor2.set(ControlMode.Follower, master.getDeviceID());
    motor3.set(ControlMode.Follower, master.getDeviceID());

    master.configMotionAcceleration(PIDConstants.MAX_ACCELERATION, Robot.timeoutMs);
    master.configMotionCruiseVelocity(PIDConstants.MAX_VELOCITY, Robot.timeoutMs);
    master.config_kP(0, PIDConstants.k_p);
    master.config_kI(0, PIDConstants.k_i);
    master.config_kD(0, PIDConstants.k_d);
    master.config_kF(0, PIDConstants.k_f);

    master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    currentPosition = LiftPositions.LOW;
    
    isMoving = false;
  }

  public void setManualLift(double power) {
    master.set(ControlMode.PercentOutput, power);
  }

  public void setLiftMotor(int id, double power) {
    if(id == 1) {
      master.set(ControlMode.PercentOutput, power);
    } else if(id == 2) {
      motor2.set(ControlMode.PercentOutput, power);
    } else if(id == 3) {
      motor3.set(ControlMode.PercentOutput, power);
    }
  }

  public int getLiftMotorID(int id) {
    if(id == 1) {
      return master.getDeviceID();
    } else if(id == 2) {
      return motor2.getDeviceID();
    } else if(id == 3) {
      return motor3.getDeviceID();
    }
    return 0;
  }

  public void configLiftClimb(){
    master.config_kF(0, PIDConstants.CLIMB_kF, Robot.timeoutMs);
    master.config_kP(0, PIDConstants.CLIMB_kP, Robot.timeoutMs);
    master.config_kI(0, PIDConstants.CLIMB_kI, Robot.timeoutMs);
    master.config_kD(0, PIDConstants.CLIMB_kD, Robot.timeoutMs);
    // master.configMotionCruiseVelocity(LiftPidConstants.HANG_VEL, Robot.timeoutMs);
    // master.configMotionAcceleration(LiftPidConstants.HANG_ACCEL, Robot.timeoutMs);

  }

  public void motionMagicLift(int pos) {
    master.set(ControlMode.MotionMagic, pos);
    SmartDashboard.putNumber("Encoder Target", pos);
  }
  public void pidLift(int pos){
    master.set(ControlMode.Position, pos);
  }

  public void resetEncoder() {
    master.setSelectedSensorPosition(0);
  }

  public int getRawEncoder() {
    return master.getSelectedSensorPosition();
  }

  public void moveLiftToPos(LiftPositions pos) {
    motionMagicLift(getLiftPositionEncoders(pos));
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
    motor2.set(ControlMode.Follower, master.getDeviceID());
    motor3.set(ControlMode.Follower, master.getDeviceID());
  }

  public void setPercentOutput() {
    motor2.set(ControlMode.PercentOutput, 0);
    motor3.set(ControlMode.PercentOutput, 0);
  }

  public void setIsMoving(boolean isMoving) {
    this.isMoving = isMoving;
  }

  public boolean getIsMoving() {
    return isMoving;
  }

  public int getLiftPositionEncoders(LiftPositions pos) {
    switch(pos) {
      case LOW:
        return getTicksFromDistance(LiftTargetInches.INTAKE);
      case CARGO_SHIP:
        return getTicksFromDistance(LiftTargetInches.CARGO_SHIP);
      case CARGO_ROCKET_LEVEL_ONE:
        return getTicksFromDistance(LiftTargetInches.CARGO_ROCKET_LEVEL_ONE);
      case CARGO_ROCKET_LEVEL_TWO:
        return getTicksFromDistance(LiftTargetInches.CARGO_ROCKET_LEVEL_TWO);
      case CARGO_ROCKET_LEVEL_THREE:
        return getTicksFromDistance(LiftTargetInches.CARGO_ROCKET_LEVEL_THREE);
      case CARGO_LOADING_STATION:
        return getTicksFromDistance(LiftTargetInches.CARGO_LOADING_STATION);
      case HATCH_MID:
        return getTicksFromDistance(LiftTargetInches.HATCH_MID);
      case HATCH_HIGH:
        return getTicksFromDistance(LiftTargetInches.HATCH_HIGH);
      case CLIMB_HAB_TWO:
        return LiftTargetEncoderTicks.CLIMB_HAB_TWO;
      case CLIMB_HAB_THREE:
        return LiftTargetEncoderTicks.CLIMB_HAB_THREE;    
    }
    return 0;
  }

  public void setLiftPID() {
    master.configMotionAcceleration(PIDConstants.MAX_ACCELERATION, Robot.timeoutMs);
    master.configMotionCruiseVelocity(PIDConstants.MAX_VELOCITY, Robot.timeoutMs);
    master.config_kP(0, PIDConstants.k_p);
    master.config_kI(0, PIDConstants.k_i);
    master.config_kD(0, PIDConstants.k_d);
    master.config_kF(0, PIDConstants.k_f);
  }

  public boolean withinTolerance(LiftPositions target) {
    return (Math.abs(getLiftPositionEncoders(target) - getRawEncoder()) < LiftTargetEncoderTicks.TOLERANCE) ? true: false;
  }

  public int getVel() {
    return master.getSelectedSensorVelocity(0);
  }

  public int getTicksFromDistance(double distance) {
    return (int) (distance / LiftTargetEncoderTicks.DISTANCE_PER_PULSE);
  }

  public double getDistanceFromTicks() {
    return LiftTargetEncoderTicks.DISTANCE_PER_PULSE * getRawEncoder();
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
