/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Robot;

/**
 * Add your docs here.
 */


public class JackSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static enum JackStates {
    DEPLOYED,
    STORED,
    INBETWEEN,
    MOVING
  }

  public static class JackEncoderConstants{
    public static int UP_STATE = 0;
    public static int DOWN_STATE_LEVEL_3 = 21740;
    public static int DOWN_STATE_LEVEL_2 =  (int)(6.0/(1.2*Math.PI/4096) + 605);
    public static int ABS_TOL = 50;
    public static double DRIVETRAIN_RATIO = 0.4;
  }

  public static class JackConstants{
    public static final double UP_kF = 0.3;
    public static final double DOWN_kF = 0.22755555555;
    public static final double UP_kP = 0.4;
    public static final double DOWN_kP = 2.5;
    public static final double UP_kI = 0;
    public static final double DOWN_kI = 0;
    public static final double UP_kD = 0;
    public static final double DOWN_kD = 0;
    public static final int UP_VEL = 7000;
    public static final int DOWN_VEL = 2500;//1800
    public static final int UP_ACCEL = 12000;//150
    public static final int DOWN_ACCEL = 1800;

  }

  private TalonSRX jackMotor;
  private TalonSRX jackDriveMotor;
  private DigitalInput botHal;
  private DigitalInput topHal;
  public Solenoid forkDeploy;

  public JackSubsystem(){
    jackMotor = new TalonSRX(RobotMap.Jacks.JACK_MOTOR);
    jackMotor.setInverted(false);
    jackMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Robot.timeoutMs);
    jackMotor.setSensorPhase(true);
    jackMotor.setNeutralMode(NeutralMode.Brake);
    jackMotor.configContinuousCurrentLimit(40,30);
    jackMotor.configPeakCurrentDuration(2000,30);
    jackMotor.configPeakCurrentLimit(60,30);
    jackMotor.enableCurrentLimit(true);
    jackMotor.selectProfileSlot(0, 0);
    forkDeploy = new Solenoid(4, RobotMap.Jacks.FORK_DEPLOY);
    jackDriveMotor = new TalonSRX(RobotMap.Jacks.JACK_DRIVE_MOTOR);
    botHal = new DigitalInput(RobotMap.Jacks.DOWN_HAL);
    topHal = new DigitalInput(RobotMap.Jacks.UP_HAL);
    releaseForks(false);
  }

  public void setJackDriveMotor(double pow){
    jackDriveMotor.set(ControlMode.PercentOutput, pow);
  }
  public void releaseForks(boolean isOut){
    forkDeploy.set(isOut);
  }
  public void setJackMotor(double pow){
    jackMotor.set(ControlMode.PercentOutput, pow);
  }
  public int getJackEncoder(){
    return jackMotor.getSelectedSensorPosition();
  }

  // TODO: ERROR
  public double getJackVel(){
    return 0.0;
    // return jackMotor.getSelectedSensorVelocity();
  }
  public void setJackMotorMP(double pos){
    jackMotor.set(ControlMode.MotionMagic, pos);
  }
  public void resetJackEncoder(){
    jackMotor.setSelectedSensorPosition(0);
  }
  public boolean isJackAtBottom(){
    return !botHal.get();
  }
  public boolean isJackAtTop(){
    return !topHal.get();
  }
  public void setJackMPVals(boolean isDown){
    if(isDown){
      jackMotor.config_kF(0, JackConstants.DOWN_kF,Robot.timeoutMs);
      jackMotor.config_kP(0, JackConstants.DOWN_kP,Robot.timeoutMs);
      jackMotor.config_kI(0, JackConstants.DOWN_kI,Robot.timeoutMs);
      jackMotor.config_kD(0, JackConstants.DOWN_kD,Robot.timeoutMs);
      jackMotor.configMotionCruiseVelocity(JackConstants.DOWN_VEL,Robot.timeoutMs);
      jackMotor.configMotionAcceleration(JackConstants.DOWN_ACCEL,Robot.timeoutMs);
    }else{
      jackMotor.config_kF(0, JackConstants.UP_kF,Robot.timeoutMs);
      jackMotor.config_kP(0, JackConstants.UP_kP,Robot.timeoutMs);
      jackMotor.config_kI(0, JackConstants.UP_kI,Robot.timeoutMs);
      jackMotor.config_kD(0, JackConstants.UP_kD,Robot.timeoutMs);
      jackMotor.configMotionCruiseVelocity(JackConstants.UP_VEL,Robot.timeoutMs);
      jackMotor.configMotionAcceleration(JackConstants.UP_ACCEL,Robot.timeoutMs);
    }
  }

  public void smartdashCurrent() {
    SmartDashboard.putNumber("Jack Current", jackMotor.getOutputCurrent());
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
    jackMotor.set(ControlMode.PercentOutput, 0);
    jackDriveMotor.set(ControlMode.PercentOutput, 0);
  }
}
