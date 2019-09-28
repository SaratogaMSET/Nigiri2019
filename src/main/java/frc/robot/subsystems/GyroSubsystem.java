/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.util.FishyMath;

/**
 * Add your docs here.
 */
public class GyroSubsystem extends Subsystem implements ILogger {
  /**
   * Add your docs here.
   */

  public static class GyroStraightConstants {
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double cumError = 0;
    public static double lastError = 0;
  }

  public double lastPos;
  public double lastAccelX;
  public double lastAccelY;
  public boolean collision;
  public AHRS gyro;

  // PID
  public PIDController gyroPIDController;
  public Double gyroPIDOutput = 0.0; // Normalized output (-1.0 to 1.0) to feed. CLOCKWISE  power output. A positive value means that the robot needs to turn to the right to hit the angle target.

  public PIDController driverGyroPID;
  public Double driverPIDOutput = 0.0; 

  public GyroSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);
    lastPos = getGyroAngle();
    lastAccelX = getLinearAccelX();
    lastAccelY = getLinearAccelY();
    collision = false;

    gyroPIDController = new PIDController(0.05, 0.0, 0.6, new PIDSource(){
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {}
      @Override
      public PIDSourceType getPIDSourceType() { return PIDSourceType.kDisplacement; }
    
      @Override
      public double pidGet() {
        return FishyMath.boundThetaNeg180to180(gyro.getAngle());
      }
    }, 
    new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        gyroPIDOutput = output;
      }
    }, 0.01);
    gyroPIDController.setInputRange(-180.0f, 180.0f);
    gyroPIDController.setOutputRange(-1.0, 1.0);
    gyroPIDController.setAbsoluteTolerance(1.0);
    gyroPIDController.setContinuous(true);
    gyroPIDController.disable();

    driverGyroPID = new PIDController(0.0002, 0.0, 0.01, 1.0/(2*Math.PI),
    new PIDSource(){
      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {}
      @Override
      public PIDSourceType getPIDSourceType() { return PIDSourceType.kRate; }
    
      @Override
      public double pidGet() {
        return gyro.getRate();
      }
    }, new PIDOutput(){
      @Override
      public void pidWrite(double output) {
        driverPIDOutput = output;
      }
    }, 0.05);
    driverGyroPID.setInputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    driverGyroPID.setOutputRange(-1.0, 1.0);
    driverGyroPID.setAbsoluteTolerance(1.0);
    driverGyroPID.setContinuous(false);
    driverGyroPID.disable();

    Shuffleboard.getTab("Drivetrain").add("GYRO ANGLE PID", gyroPIDController);
    Shuffleboard.getTab("Drivetrain").add("DRIVER GYRO PID", driverGyroPID);
    Shuffleboard.getTab("Drivetrain").add("GYRO OUTPUT", new Sendable(){
    
      @Override
      public void setSubsystem(String subsystem) {}
    
      @Override
      public void setName(String name) {}
    
      @Override
      public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.addDoubleProperty("OUTPUT", new DoubleSupplier(){
        
          @Override
          public double getAsDouble() {
            // TODO Auto-generated method stub
            return driverPIDOutput;
          }
        }, null);
        
      }
    
      @Override
      public String getSubsystem() {
        // TODO Auto-generated method stub
        return null;
      }

      @Override
      public String getName() {
        // TODO Auto-generated method stub
        return "OUTPUT";
      }
    });

  }

  public double getFusedHeading() {
    return gyro.getFusedHeading();
  }

  public synchronized double getGyroAngle() {
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

  public double getGyroStraightPIDOutput(double error) {
    double p = GyroStraightConstants.kp * error;
    GyroStraightConstants.cumError += error;
    double i = GyroStraightConstants.ki * GyroStraightConstants.cumError;
    double dError = error - GyroStraightConstants.lastError;
    GyroStraightConstants.lastError = error;
    double d = GyroStraightConstants.kd * dError;

    return p + i + d;
  }

  public void PIDReset() {
    GyroStraightConstants.cumError = 0;
    GyroStraightConstants.lastError = 0;
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

  public double getGyroPIDOutput() {
    return gyroPIDOutput;
  }
}
