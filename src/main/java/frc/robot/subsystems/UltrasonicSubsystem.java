/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class UltrasonicSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private AnalogInput ultrasonicLeft;
  private AnalogInput ultrasonicRight;
  private double scalingFactor;

  public UltrasonicSubsystem() {
    ultrasonicLeft = new AnalogInput(RobotMap.Ultrasonic.ULTRASONIC_LEFT);
    ultrasonicRight = new AnalogInput(RobotMap.Ultrasonic.ULTRASONIC_RIGHT);
    scalingFactor = 5.0/512;
  }

  public double getLeftRawVoltage() {
    return ultrasonicLeft.getVoltage();
  }

  public double getLeftDistance() {
    return ultrasonicLeft.getVoltage()/scalingFactor;
  }

  public double getRightRawVoltage() {
    return ultrasonicRight.getVoltage();
  }

  public double getRightDistance() {
    return getRightRawVoltage()/scalingFactor;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
