/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Add your docs here.
 */
public class GyroSubsystem extends Subsystem implements ILogger, PIDOutput {
  /**
   * Add your docs here.
   */

  public double lastPos;
  public double lastAccelX;
  public double lastAccelY;
  public boolean collision;
  public AHRS gyro;

  // PID
  public PIDController gyroPIDController;
  private Double gyroPIDOutput; // Normalized output (-1.0 to 1.0) to feed. CLOCKWISE  power output. A positive value means that the robot needs to turn to the right to hit the angle target.

  public GyroSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);
    lastPos = getGyroAngle();
    lastAccelX = getLinearAccelX();
    lastAccelY = getLinearAccelY();
    collision = false;

    gyroPIDController = new PIDController(0.0, 0.0, 0.0, gyro, this);
    gyroPIDController.setInputRange(-180.0f, 180.0f);
    gyroPIDController.setAbsoluteTolerance(2.0);
    gyroPIDController.setContinuous(true);
    gyroPIDController.disable();

    Shuffleboard.getTab("Drivetrain").add(gyroPIDController);
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
    return gyro.getWorldLinearAccelY();
  }

  public double getJerkX() {
    double jerkX = Math.abs(getLinearAccelX() - lastAccelX);
    lastAccelX = getLinearAccelX();
    return jerkX;
  }

  public double getLinearAccelY() {
    return gyro.getWorldLinearAccelX();
  }

  public double getJerkY() {
    double jerkY = Math.abs(getLinearAccelY() - lastAccelY);
    lastAccelY = getLinearAccelY();
    return jerkY;
  }

  public boolean collision(double threshold) {
    if (!collision) {
      collision = getJerkX() > threshold || getJerkY() > threshold;
    }
    return collision;
  }

  public void resetCollision() {
    lastAccelX = getLinearAccelX();
    lastAccelY = getLinearAccelY();
    collision = false;
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

  /**
   * Output for the PID Controller. Writes to a subsystem-owned variable of the offset to add to the drivetrain motors. If Robot activates the Gyro PID, it should read from the variable to actuate the motors.
   */
  @Override
  public void pidWrite(double output) {
    this.gyroPIDOutput = output;
  }

  public double getGyroPIDOutput() {
    return gyroPIDOutput;
  }
}
