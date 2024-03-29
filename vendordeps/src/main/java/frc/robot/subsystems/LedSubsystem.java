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