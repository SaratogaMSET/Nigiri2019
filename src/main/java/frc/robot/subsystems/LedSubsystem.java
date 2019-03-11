/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class LedSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Spark led = new Spark(0);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  
  public static class ledcodes{
    double led1 = 0.57 ;
    double led2 = 0.59;
    double led3 = 0.61;
    double led4 = 0.63 ;
    double led5 = 0.65 ;
    double led6 = 0.67;
    double led7 = 0.69;
    double led8 = 0.71;
    double led9 = 0.73;
    double led10 = 0.75;
    double led11 = 0.77 ;
    double led12 = 0.79 ;
    double led13 = 0.81;
    double led14 = 0.83;
    double led15 = 0.85 ;
    double led16 = 0.87;
    double led17 = 0.89;
    double led18 = 0.91;
    double led19 = 0.93;
    double led20 = 0.95;
  }

  public void blink(int seconds){
    led.set(.11); //green
    Timer.delay(seconds);
    led.set(.99);
  }

  public void solid(int seconds){
    led.set(.59); //solid color lime
    Timer.delay(seconds);
    led.set(.99);
  }

  public void chase(int seconds){
    led.set(.31); //lightchase red
    Timer.delay(seconds);
    led.set(.99);
  }

  @Override
  public void diagnosticShuffleboard() {
    
  }

  @Override
  public void essentialShuffleboard() {
    
  }

}
