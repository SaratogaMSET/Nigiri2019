/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * Add your docs here.
 */
public class CameraSubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  //public static UsbCamera microsoft;

  public static UsbCamera jevois;

  public static AxisCamera axis;

  public static double max=0;

  public CameraSubsystem(){

<<<<<<< HEAD
    //microsoft = CameraServer.getInstance().startAutomaticCapture();
    //microsoft.setResolution(320, 240);

    jevois = CameraServer.getInstance().startAutomaticCapture();
    jevois.setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);

    //axis = CameraServer.getInstance().addAxisCamera("10.6.49.11");
=======
    // microsoft = CameraServer.getInstance().startAutomaticCapture();
    // microsoft.setResolution(320, 240);

    jevois = CameraServer.getInstance().startAutomaticCapture();
    jevois.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);

    // axis = CameraServer.getInstance().addAxisCamera("10.6.49.11");
>>>>>>> 5a288e2772560c8f6c64ad081dc80d9c9bb3129e
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static double maxBandwidth(VideoCamera cam){
<<<<<<< HEAD
    // CameraServerJNI.setTelemetryPeriod(100);
=======
    //CameraServerJNI.setTelemetryPeriod(100);
>>>>>>> 5a288e2772560c8f6c64ad081dc80d9c9bb3129e
    // if(cam.getActualDataRate()>max){
    //   max=cam.getActualDataRate();
    // }
    return axis.getActualDataRate();
  }

  @Override
  public void diagnosticShuffleboard() {
    
  }

  @Override
  public void essentialShuffleboard() {
    
  }
}
