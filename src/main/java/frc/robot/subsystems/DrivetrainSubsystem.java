/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.FileWriter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.util.FishyMath;
import frc.robot.util.drivers.LazySparkMax;
import frc.robot.util.drivers.SparkMaxFactory;
import frc.robot.util.pid.AdvancedPIDController;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.PathfinderJNI;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;


/**
 * Add your docs here.
 */
public class DrivetrainSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // motors[0] is the right master, motors [1,2] right followers
  // motors[3] is the left master, motors [4,5] left followers

  // wouldn't want to tip, now would we?
  // these are the values that go into pathfinder
  public static final double ROBOT_TARGET_MAX_VELOCITY = 14.0; // ft/s
  public static final double ROBOT_TARGET_MAX_ACCELERATION = 20.0; // ft/s^2

  public LazySparkMax leftMaster, rightMaster, leftSlave, rightSlave;
  public TalonSRX leftEncoderSRX, rightEncoderSRX; 

  public boolean isPathRunning;
  public int motor;

  public static final double WHEEL_DIAMETER = (4.06/12.0); // feet
  public static final int TICKS_PER_REV = 4096;
  public static final double WHEELBASE_FEET = 2.1804; // feet
  public static final double EMPIRICAL_WHEELBASE_FEET = 2.1804; // feet

  public double pidInputPower = 0.0;
  public PIDController joystickPID = new PIDController(0.04, 0.0, 0.005, 0.5, new PIDSource(){
      @Override
      public double pidGet() {
          return pidInputPower;
          // return MAX_VEL * OI.getInstance().driver.getArcadeLeft();
      }
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {}
      @Override
      public PIDSourceType getPIDSourceType() {return PIDSourceType.kRate;}
  }, new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        pidInputPower = output;
      }
  }, 0.02);


  public DrivetrainSubsystem() {
    leftMaster = SparkMaxFactory.createDefaultSparkMax(1);
    configureSpark(leftMaster, true, true);

    leftSlave = SparkMaxFactory.createPermanentSlaveSparkMax(2, leftMaster);
    configureSpark(leftSlave, true, true);

    rightMaster = SparkMaxFactory.createDefaultSparkMax(3);
    configureSpark(rightMaster, false, true);

    rightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(4, rightMaster);
    configureSpark(rightSlave, false, true);


    leftEncoderSRX = new TalonSRX(23);
    leftEncoderSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftEncoderSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
    leftEncoderSRX.configVelocityMeasurementWindow(16);


    rightEncoderSRX = new TalonSRX(10);
    rightEncoderSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightEncoderSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
    rightEncoderSRX.configVelocityMeasurementWindow(16);


    joystickPID.setInputRange(-1, 1);
    joystickPID.setOutputRange(-1, 1);
    joystickPID.setAbsoluteTolerance(0.01);
    joystickPID.setContinuous(false);
    joystickPID.disable();
    // motors = new TalonSRX[6];
    // for (int i = 0; i < motors.length; i++) {
    //   motors[i] = new TalonSRX(RobotMap.Drivetrain.DRIVETRAIN_MOTOR_PORTS[i]);
    //   motors[i].configNominalOutputForward(0.08, 200);
    //   motors[i].configNominalOutputReverse(-0.08, 200);
    //   motors[i].configPeakOutputForward(1, 200);
    //   motors[i].configPeakOutputReverse(-1, 200);
    // }

    // motors[0].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 200);
    // motors[0].config_kF(0, 0.17, 200);
    // motors[0].config_kP(0, 0.0, 200);
    // motors[0].config_kI(0, 0.0, 200);
    // motors[0].config_IntegralZone(0, (int) (FishyMath.rpm2talonunits(FishyMath.fps2rpm(1.0))), 200);
    // motors[0].config_kD(0, 0.0, 200);
    // motors[0].configAllowableClosedloopError(0, 0, 200);
    // motors[0].configClosedLoopPeriod(0, 1, 200);
    // motors[0].configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 200);
    // motors[0].configVelocityMeasurementWindow(16, 200);
    // motors[0].selectProfileSlot(0, 0);
    // // motors[0].config_kF(0, 0.23);
    // // motors[0].config_kP(0, 2.2);
    // // motors[0].config_kI(0, 0.00);
    // // motors[0].config_kD(0, 45.0);


    // motors[3].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 200);
    // motors[3].config_kF(0, 0.17);
    // motors[3].config_kP(0, 0.0);
    // motors[3].config_kI(0, 0.0);
    // motors[3].config_IntegralZone(0, (int) (FishyMath.rpm2talonunits(FishyMath.fps2rpm(1.0))));
    // motors[3].config_kD(0, 0.0);
    // motors[3].configAllowableClosedloopError(0, 0, 200);
    // motors[3].configClosedLoopPeriod(0, 1, 200);
    // motors[3].configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, 200);
    // motors[3].configVelocityMeasurementWindow(16, 200);
    // motors[3].selectProfileSlot(0, 0);


    // // follow right master
    // motors[1].set(ControlMode.Follower, motors[0].getDeviceID());
    // motors[2].set(ControlMode.Follower, motors[0].getDeviceID());

    // // follow left master
    // motors[4].set(ControlMode.Follower, motors[3].getDeviceID());
    // motors[5].set(ControlMode.Follower, motors[3].getDeviceID());

    // // invert right side DT motors
    // motors[0].setInverted(true);
    // motors[3].setInverted(false);

    // motors[1].setInverted(InvertType.FollowMaster);
    // motors[2].setInverted(InvertType.FollowMaster);
    // motors[4].setInverted(InvertType.FollowMaster);
    // motors[5].setInverted(InvertType.FollowMaster);
  }

  private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
    sparkMax.setInverted(!left);
    sparkMax.enableVoltageCompensation(12.0);
}

  public void changeBrakeCoast(boolean isBrake) {
    if (isBrake) {
      leftMaster.setIdleMode(IdleMode.kBrake);
      rightMaster.setIdleMode(IdleMode.kBrake);
      rightSlave.setIdleMode(IdleMode.kBrake);
      leftSlave.setIdleMode(IdleMode.kBrake);
    }
    else {
      leftMaster.setIdleMode(IdleMode.kCoast);
      rightMaster.setIdleMode(IdleMode.kCoast);
      rightSlave.setIdleMode(IdleMode.kCoast);
      leftSlave.setIdleMode(IdleMode.kCoast);
    }
  }

  public void rawDrive(double left, double right) {
    leftMaster.set(left);
    rightMaster.set(right);
  }

  public int getRawLeftEncoder() {
    return leftEncoderSRX.getSelectedSensorPosition();
  }

  public void driveFwdRotate(double fwd, double rot){
		double left = fwd + rot, right = fwd - rot;
		double max = Math.max(1, Math.max(Math.abs(left), Math.abs(right)));
		left /= max;
		right /= max;

		rawDrive(left, right);
  }

  public int getRawRightEncoder() {
    return rightEncoderSRX.getSelectedSensorPosition();
  }

  public synchronized double getRightEncoderVelocity() {
    return FishyMath.rpm2fps(FishyMath.talaonunits2rpm(rightEncoderSRX.getSelectedSensorVelocity()));
  }

  public synchronized double getLeftEncoderVelocity() {
    return FishyMath.rpm2fps(FishyMath.talaonunits2rpm(leftEncoderSRX.getSelectedSensorVelocity()));
  }

  public synchronized double getRightEncoderDistance() {
    return (rightEncoderSRX.getSelectedSensorPosition() / (double) TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI;
  }

  public synchronized double getLeftEncoderDistance() {
    return (leftEncoderSRX.getSelectedSensorPosition() / (double) TICKS_PER_REV) * WHEEL_DIAMETER * Math.PI;
  }

  public void resetEncoders() {
    leftEncoderSRX.setSelectedSensorPosition(0);
    rightEncoderSRX.setSelectedSensorPosition(0);
  }

  public void testDrivetrain(double fwd, boolean changeMotor, boolean resetEncoders) {
    if (resetEncoders) {
      resetEncoders();
    }
    if (changeMotor) {
      if (motor < 6) {
        motor++;
      }
      else {
        motor = 0;
      }
    }

    if (motor < 6) {
      // SmartDashboard.putNumber("Motor", motors[motor].getDeviceID());
      // testMotor(fwd);
    }
    else {
      // SmartDashboard.putString("Motor", "all");
      testAllMotors(fwd);
    }
    // SmartDashboard.putNumber("Left Encoder", getRawLeftEncoder());
    // SmartDashboard.putNumber("Right Encoder", getRawRightEncoder());
  }

  public void testMotor(int num, double fwd) {
    // motors[num].set(ControlMode.PercentOutput, fwd);
  }

  public void testAllMotors(double fwd) {
    leftMaster.set(fwd);
    leftSlave.set(fwd);
    rightMaster.set(fwd);
    rightSlave.set(fwd);
  }

  public void resetControlMode() {
    // for (int i = 0; i < motors.length; i++) {
    //   motors[i].set(ControlMode.PercentOutput, 0);
    // }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void diagnosticShuffleboard() {
    ShuffleboardTab drive = Shuffleboard.getTab("Drive");
    // drive.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kTextView);
    // drive.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kTextView);
  }

  @Override
  public void essentialShuffleboard() {
    ShuffleboardTab drive = Shuffleboard.getTab("Drive");
    // drive.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kTextView);
    // drive.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kTextView);
  }

  @Override
  public void stopAll() {
    rawDrive(0.0, 0.0);
  }
}
