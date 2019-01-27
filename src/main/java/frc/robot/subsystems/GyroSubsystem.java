/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class GyroSubsystem extends Subsystem implements ILogger {
  /**
   * Add your docs here.
   */

  public double lastPos;
  public double lastAccelX;
  public double lastAccelY;
  public AHRS gyro;

  public GyroSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);
    lastPos = getGyroAngle();
    lastAccelX = getLinearAccelX();
    lastAccelY = getLinearAccelY();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
    lastPos = gyro.getAngle();
  }

  public double getRelativeAngle() {
    double relativeAngle = gyro.getAngle() - lastPos;
    lastPos = gyro.getAngle();
    return relativeAngle;
  }

  public double getLinearAccelX() {
    return gyro.getWorldLinearAccelX();
  }

  public double getJerkX() {
    double jerkX = Math.abs(getLinearAccelX() - lastAccelX);
    lastAccelX = getLinearAccelX();
    return jerkX;
  }

  public double getLinearAccelY() {
    return gyro.getWorldLinearAccelY();
  }

  public double getJerkY() {
    double jerkY = Math.abs(getLinearAccelY() - lastAccelY);
    lastAccelY = getLinearAccelY();
    return jerkY;
  }

  public boolean collision(double threshold) {
    return getJerkX() > threshold || getJerkY() > threshold;
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
