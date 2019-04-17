/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotState;

/**
 * Add your docs here.
 */
public class LiftSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static enum LiftPositions {
    TRUE_BOTTOM,
    LOW,
    CARGO_SHIP,
    CARGO_ROCKET_LEVEL_ONE,
    CARGO_ROCKET_LEVEL_TWO,
    CARGO_ROCKET_LEVEL_THREE,
    CARGO_LOADING_STATION,
    HATCH_MID,
    HATCH_HIGH,
    CLIMB_HAB_TWO,
    CLIMB_HAB_THREE,
    MANUAL,
    PREP_CLIMB_1,
    PREP_CLIMB_2,
    CLIMB_HAB_TWO_TOL,
    CLIMB_HAB_THREE_TOL,
    LOW_HATCH,
    AUTO_CARGO_SHIP_HATCH
  }

  public static class LiftEncoderConstants {
    public static final int CLIMB_HAB_TWO = 3300;
    public static final int CLIMB_HAB_TWO_TOL = 5000;
    public static final int CLIMB_HAB_THREE = 13750;
    public static final int CLIMB_HAB_THREE_TOL = 15000;
    public static final double LIFT_TICKS_PER_JACK_TICK = 1.2/1.75; //for every tick of jack go this much lift
    public static final double DISTANCE_PER_PULSE = 1.75 * 2 * Math.PI / 4096.0;
    public static final int TOLERANCE = 75;
    public static final int STATE_TOLERANCE = 125;
    public static final int VELOCITY_THRESH = 50;
  }

  public static class LiftDistanceConstants {
    public static final double TRUE_BOTTOM = -0.25;
    public static final double LOW_HATCH = 5;
    public static final double INTAKE = 0;
    public static final double CARGO_SHIP = 30;
    public static final double CARGO_ROCKET_LEVEL_ONE = 18.5;
    public static final double CARGO_ROCKET_LEVEL_TWO = 46.04;
    public static final double CARGO_ROCKET_LEVEL_THREE = 67.5;
    public static final double CARGO_LOADING_STATION = 32.5;
    public static final double HATCH_MID = 27.88 + 4;
    public static final double HATCH_HIGH = 54.4 + 4;
    public static final double PREP_CLIMB_1 = 31.7;
    public static final double PREP_CLIMB_2 = 28.23;
    public static final double AUTO_CARGO_SHIP_HATCH = 9;
  }

  public static class LiftPidConstants {
    // Use feedBACK only for the downwards lift pushing for the climb.
    public static final double CLIMB_kF = 0.59;
    public static final double CLIMB_kP = 1.4; //1.6
    public static final double CLIMB_kI = 0.0;
    public static final double CLIMB_kD = 0.01;
  }

  public static class PIDConstants {
    public static final double k_f = 0.15; //0.295
    public static double k_p = 0.5; //0.8
    public static double k_i = 0.0;
    public static double k_d = 15;
    public static final int MAX_ACCELERATION = 13000; //measured 40000-70000
    public static final int MAX_VELOCITY = 4250; // measured 4500
  }

  private TalonSRX motor1;
  private TalonSRX motor2;
  private TalonSRX motor3;
  private DigitalInput bottomHal;
  private DigitalInput topHal;
  private boolean isMoving;
  private boolean lastBottomHal;

  public static boolean isRunningLiftPID = false;

  public LiftSubsystem() {
    motor1 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_1_PORT);
    motor2 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_2_PORT);
    motor3 = new TalonSRX(RobotMap.Lift.LIFT_MOTOR_3_PORT);

    bottomHal = new DigitalInput(RobotMap.Lift.BOTTOM_HAL_EFFECT);
    // topHal = new DigitalInput(RobotMap.Lift.TOP_HAL_EFFECT);

    motor1.configNominalOutputForward(0, Robot.timeoutMs);
    motor1.configNominalOutputReverse(0, Robot.timeoutMs);
    motor1.configPeakOutputForward(1, Robot.timeoutMs);
    motor1.configPeakOutputReverse(-1, Robot.timeoutMs);

    motor2.configNominalOutputForward(0, Robot.timeoutMs);
    motor2.configNominalOutputReverse(0, Robot.timeoutMs);
    motor2.configPeakOutputForward(1, Robot.timeoutMs);
    motor2.configPeakOutputReverse(-1, Robot.timeoutMs);

    motor3.configNominalOutputForward(0, Robot.timeoutMs);
    motor3.configNominalOutputReverse(0, Robot.timeoutMs);
    motor3.configPeakOutputForward(1, Robot.timeoutMs);
    motor3.configPeakOutputReverse(-1, Robot.timeoutMs);

    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor3.set(ControlMode.Follower, motor1.getDeviceID());

    motor1.selectProfileSlot(0, 0);
    motor1.configMotionAcceleration(PIDConstants.MAX_ACCELERATION, Robot.timeoutMs);
    motor1.configMotionCruiseVelocity(PIDConstants.MAX_VELOCITY, Robot.timeoutMs);
    motor1.config_kP(0, PIDConstants.k_p);
    motor1.config_kI(0, PIDConstants.k_i);
    motor1.config_kD(0, PIDConstants.k_d);
    motor1.config_kF(0, PIDConstants.k_f);

    motor1.setNeutralMode(NeutralMode.Coast);
    motor2.setNeutralMode(NeutralMode.Coast);
    motor3.setNeutralMode(NeutralMode.Coast);

    motor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    isMoving = false;
    lastBottomHal = false;
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
    motor1.configMotionAcceleration(PIDConstants.MAX_ACCELERATION, Robot.timeoutMs);
    motor1.configMotionCruiseVelocity(PIDConstants.MAX_VELOCITY, Robot.timeoutMs);
    motor1.config_kP(0, LiftPidConstants.CLIMB_kP);
    motor1.config_kI(0, PIDConstants.k_i);
    motor1.config_kD(0, LiftPidConstants.CLIMB_kD);
    motor1.config_kF(0, PIDConstants.k_f);
    // motor1.configMotionCruiseVelocity(LiftPidConstants.HANG_VEL, Robot.timeoutMs);
    // motor1.configMotionAcceleration(LiftPidConstants.HANG_ACCEL, Robot.timeoutMs);

  }

  public void motionMagicLift(int pos) {
    motor1.set(ControlMode.MotionMagic, pos);
    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor3.set(ControlMode.Follower, motor1.getDeviceID());
    // SmartDashboard.putNumber("Encoder Target", pos);
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
      case TRUE_BOTTOM:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.TRUE_BOTTOM));
        break;
      case LOW:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.INTAKE));
        break;
      case LOW_HATCH:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.LOW_HATCH));
        break;
      case CARGO_SHIP:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.CARGO_SHIP));
        break;
      case CARGO_ROCKET_LEVEL_ONE:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.CARGO_ROCKET_LEVEL_ONE));
        break;
      case CARGO_ROCKET_LEVEL_TWO:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.CARGO_ROCKET_LEVEL_TWO));
        break;
      case CARGO_ROCKET_LEVEL_THREE:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.CARGO_ROCKET_LEVEL_THREE));
        break;
      case CARGO_LOADING_STATION:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.CARGO_LOADING_STATION));
        break;
      case HATCH_MID:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.HATCH_MID));
        break;
      case HATCH_HIGH:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.HATCH_HIGH));
        break;
      case CLIMB_HAB_TWO:
        motionMagicLift(LiftEncoderConstants.CLIMB_HAB_TWO);
        break;
      case CLIMB_HAB_THREE:
        motionMagicLift(LiftEncoderConstants.CLIMB_HAB_THREE);
        break;   
      case PREP_CLIMB_1:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.PREP_CLIMB_1));
        break;
      case PREP_CLIMB_2:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.PREP_CLIMB_2));
        break;
      case CLIMB_HAB_TWO_TOL:
        motionMagicLift(LiftEncoderConstants.CLIMB_HAB_TWO_TOL);
        break;
      case CLIMB_HAB_THREE_TOL:
        motionMagicLift(LiftEncoderConstants.CLIMB_HAB_THREE_TOL);
        break;
      case AUTO_CARGO_SHIP_HATCH:
        motionMagicLift(getTicksFromDistance(LiftDistanceConstants.AUTO_CARGO_SHIP_HATCH));
        break;
      case MANUAL:
        // nothing
        break;
      
    }
  }

  // public boolean getTopHal() {
    // return !topHal.get();
  // }

  public boolean getBottomHal() {
    return !bottomHal.get();
  }

  public boolean isZero() {
    if(getBottomHal() && !lastBottomHal) {
      lastBottomHal = getBottomHal();
      return true;
    }
    lastBottomHal = getBottomHal();
    return false;
  }

  public void setFollowers() {
    motor2.set(ControlMode.Follower, motor1.getDeviceID());
    motor3.set(ControlMode.Follower, motor1.getDeviceID());
  }

  public void setPercentOutput() {
    motor2.set(ControlMode.PercentOutput, 0);
    motor3.set(ControlMode.PercentOutput, 0);
  }

  public boolean isMoving() {
    if(Math.abs(motor1.getSelectedSensorVelocity(0)) > LiftEncoderConstants.VELOCITY_THRESH || 
    RobotState.liftPosition != RobotState.lastLiftTarget) {
      // SmartDashboard.putNumber("Lift Velocity", motor1.getSelectedSensorVelocity(0));
      return true;
    }
    return false;
  }
 
  public int getLiftPositionEncoders(LiftPositions pos) {
    switch(pos) {
      case LOW:
        return getTicksFromDistance(LiftDistanceConstants.INTAKE);
      case CARGO_SHIP:
        return getTicksFromDistance(LiftDistanceConstants.CARGO_SHIP);
      case CARGO_ROCKET_LEVEL_ONE:
        return getTicksFromDistance(LiftDistanceConstants.CARGO_ROCKET_LEVEL_ONE);
      case LOW_HATCH:
        return getTicksFromDistance(LiftDistanceConstants.LOW_HATCH);
      case CARGO_ROCKET_LEVEL_TWO:
        return getTicksFromDistance(LiftDistanceConstants.CARGO_ROCKET_LEVEL_TWO);
      case CARGO_ROCKET_LEVEL_THREE:
        return getTicksFromDistance(LiftDistanceConstants.CARGO_ROCKET_LEVEL_THREE);
      case CARGO_LOADING_STATION:
        return getTicksFromDistance(LiftDistanceConstants.CARGO_LOADING_STATION);
      case HATCH_MID:
        return getTicksFromDistance(LiftDistanceConstants.HATCH_MID);
      case HATCH_HIGH:
        return getTicksFromDistance(LiftDistanceConstants.HATCH_HIGH);
      case CLIMB_HAB_TWO:
        return LiftEncoderConstants.CLIMB_HAB_TWO;
      case CLIMB_HAB_THREE:
        return LiftEncoderConstants.CLIMB_HAB_THREE;  
      case PREP_CLIMB_1:
        return getTicksFromDistance(LiftDistanceConstants.PREP_CLIMB_1);
      case PREP_CLIMB_2:
        return getTicksFromDistance(LiftDistanceConstants.PREP_CLIMB_2);
      case CLIMB_HAB_TWO_TOL:
        return LiftEncoderConstants.CLIMB_HAB_TWO_TOL;
      case CLIMB_HAB_THREE_TOL:
        return LiftEncoderConstants.CLIMB_HAB_THREE_TOL;
      case AUTO_CARGO_SHIP_HATCH:
        return getTicksFromDistance(LiftDistanceConstants.AUTO_CARGO_SHIP_HATCH);
      case MANUAL:
        return getRawEncoder();
    }
    return 0;
  }

  public void setLiftPID() {
    motor1.configMotionAcceleration(PIDConstants.MAX_ACCELERATION, Robot.timeoutMs);
    motor1.configMotionCruiseVelocity(PIDConstants.MAX_VELOCITY, Robot.timeoutMs);
    motor1.config_kP(0, PIDConstants.k_p);
    motor1.config_kI(0, PIDConstants.k_i);
    motor1.config_kD(0, PIDConstants.k_d);
    motor1.config_kF(0, PIDConstants.k_f);
  }

  public boolean withinTolerance(LiftPositions target) {
    return (Math.abs(getLiftPositionEncoders(target) - getRawEncoder()) < LiftEncoderConstants.TOLERANCE) ? true: false;
  }

  public boolean withinStateTolerance(LiftPositions target) {
    return (Math.abs(getLiftPositionEncoders(target) - getRawEncoder()) < LiftEncoderConstants.STATE_TOLERANCE) ? true: false;
  }

  public boolean withinStateToleranceBottom() {
    return (Math.abs(getLiftPositionEncoders(LiftPositions.LOW) - getRawEncoder()) < 400) ? true: false;
  }

  public int getVel() {
    return motor1.getSelectedSensorVelocity(0);
  }

  public int getTicksFromDistance(double distance) {
    return (int) (distance / LiftEncoderConstants.DISTANCE_PER_PULSE);
  }

  public double getDistance() {
    return LiftEncoderConstants.DISTANCE_PER_PULSE * getRawEncoder();
  }

  public boolean goingUp(LiftPositions target, LiftPositions current) {
    return (getLiftPositionEncoders(target) - getRawEncoder() > 0) ? true : false;
  }

  public double getCurrentMainMotor() {
    return motor1.getOutputCurrent();
  }

  public double getVoltageMainMotor() {
    return motor1.getMotorOutputVoltage();
  }

  public LiftPositions updateLiftPosition() {
    if(withinStateTolerance(LiftPositions.LOW)) {
      return LiftPositions.LOW;
    } else if(withinStateTolerance(LiftPositions.LOW_HATCH)) {
      return LiftPositions.LOW_HATCH;
    } else if(withinStateTolerance(LiftPositions.CARGO_ROCKET_LEVEL_ONE)) {
      return LiftPositions.CARGO_ROCKET_LEVEL_ONE;
    } else if(withinStateTolerance(LiftPositions.CARGO_ROCKET_LEVEL_TWO)) {
      return LiftPositions.CARGO_ROCKET_LEVEL_TWO;
    } else if(withinStateTolerance(LiftPositions.CARGO_ROCKET_LEVEL_THREE)) {
      return LiftPositions.CARGO_ROCKET_LEVEL_THREE;
    } else if(withinStateTolerance(LiftPositions.CARGO_LOADING_STATION)) {
      return LiftPositions.CARGO_LOADING_STATION;
    } else if(withinStateTolerance(LiftPositions.CARGO_SHIP)) {
      return LiftPositions.CARGO_SHIP;
    } else if(withinStateTolerance(LiftPositions.HATCH_MID)) {
      return LiftPositions.HATCH_MID;
    } else if(withinStateTolerance(LiftPositions.HATCH_HIGH)) {
      return LiftPositions.HATCH_HIGH;
    } else if(withinStateTolerance(LiftPositions.AUTO_CARGO_SHIP_HATCH)) {
      return LiftPositions.AUTO_CARGO_SHIP_HATCH;
    }
    return LiftPositions.MANUAL;
  }

  public void stallLift(LiftPositions pos) {
    if(Math.abs(getRawEncoder() - getLiftPositionEncoders(pos)) > LiftEncoderConstants.TOLERANCE) {
      moveLiftToPos(pos);
    }
  }

  public void smartdashCurrent() {
    SmartDashboard.putNumber("Motor 1 Current", motor1.getOutputCurrent());
    SmartDashboard.putNumber("Motor 2 Current", motor2.getOutputCurrent());
    SmartDashboard.putNumber("Motor 3 Current", motor3.getOutputCurrent());
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

  @Override
  public void stopAll() {
    setManualLift(0);
  }
}
