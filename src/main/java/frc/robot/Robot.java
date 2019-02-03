/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.*;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap.CargoDeploy;
import frc.robot.commands.DrivetrainTest;
import frc.robot.commands.GyroStraightDistancePID;
import frc.robot.commands.RunCargoDeployCommand;
import frc.robot.commands.VisionFixCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CargoDeploySubsystem;
import frc.robot.subsystems.CargoIntakeSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.DriveStraightConstants;
import frc.robot.subsystems.GyroSubsystem.GyroStraightConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI oi;

  // Subsystems
  public static CargoDeploySubsystem cargoDeploy;
  public static DrivetrainSubsystem drive;
  public static LedSubsystem led;
  public static CameraSubsystem camera;
  public static GyroSubsystem gyro;
  public static CargoIntakeSubsystem cargoIntake;

  // Vision
  public static VisionSubsystem vision;
  public static VisionFixCommand visionFixCommand;

  public final static Map<SubsystemEnum, Subsystem> loggingArr = new HashMap<>() {{
      put(SubsystemEnum.Drive, drive);
      put(SubsystemEnum.CargoDeploy, cargoDeploy);
      put(SubsystemEnum.LED, led);
      put(SubsystemEnum.Camera, camera);
      put(SubsystemEnum.Gyro, gyro);
      put(SubsystemEnum.Vision, vision);
  }};

  public static Preferences prefs;

  public static int timeoutMs = 20;

  public static TalonSRX motor1;
  public static TalonSRX motor2;
  Joystick joy1 = new Joystick(0);
  Joystick joy2 = new Joystick(1);

  public double prev_vel = 0.0; // TODO put in diagnostic

  public double max_vel = 0.0;
  public double max_accel = 0.0;
  public double lastLeftEncoder;
  public double lastRightEncoder;
  public double xLoc;
  public double yLoc;
  public double targX;
  public double targY;
  public double targH;

 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Shuffleboard.getTab("Drive").add("Time Left", Timer.getFPGATimestamp()).withSize(2, 4).withPosition(2,4)
                        .withWidget(BuiltInWidgets.kNumberBar).getEntry();
    oi = new OI();
    drive = new DrivetrainSubsystem();
    cargoDeploy = new CargoDeploySubsystem();
    cargoIntake = new CargoIntakeSubsystem();
    led = new LedSubsystem();
    //camera = new CameraSubsystem();
    gyro = new GyroSubsystem();
    try {
      vision = new VisionSubsystem();
    }
    catch(Exception e){
      SmartDashboard.putString("VISION FAILED", "1");
      e.printStackTrace();
    }
    prefs = Preferences.getInstance();

    drive.changeBrakeCoast(false);
  }
   /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specififc periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("bandwidth", camera.max);
    // System.out.println(camera.max);
    SmartDashboard.putNumber("GYRO HEADING", gyro.getGyroAngle());
    SmartDashboard.putNumber("MAX VELOCITY", max_vel = Math.max(max_vel, Math.abs(drive.leftEncoder.getRate())));    
    double curA = (Math.abs(drive.leftEncoder.getRate()) - Math.abs(prev_vel))/(0.2);
    SmartDashboard.putNumber("CURRENT ACCELERATION", curA);
    SmartDashboard.putNumber("MAX ACCELERATION", max_accel = Math.max(max_accel, Math.abs(curA)));
    prev_vel = Math.abs(drive.leftEncoder.getRate());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    drive.stopMP();
    gyro.resetGyro();
    visionFixCommand = new VisionFixCommand();
    drive.changeBrakeCoast(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    // testMotors();

    // FOR CONNOR
    // new RunCargoDeployCommand().start();
    

    //SmartDashboard.putNumber("bandwidth", camera.max);
    gyroStraightTest();

  }

  @Override
  public void testInit() {
    this.gyro.resetGyro();
    this.drive.resetEncoders();
    drive.changeBrakeCoast(true);
    drive.setTrajectory("TestPath", 1.0, 0, 0.0, 16.0, 0.01); //0.095
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();

    // TEST GYRO PID
    gyro.gyroPIDController.enable();
    double pidOut = gyro.getGyroPIDOutput();
    drive.rawDrive(pidOut, -pidOut);

  }

  
  public void sendShuffleboard(SubsystemEnum[] subs) {
    // THIS IS THROWING AN ERROR. COMMENTING OUT. - Dhruv
    /*
    if(subs[0].equals(SubsystemEnum.AllEssentials)) {
      for(Subsystem s : loggingArr.values()) {
        s.essentialShuffleboard();
      }
    }
    for(SubsystemEnum s : subs) {
      Subsystem sub = loggingArr.get(s);
      sub.diagnosticShuffleboard();
    }
    */
    SmartDashboard.putNumber("Right Encoder", drive.getRawRightEncoder());
    SmartDashboard.putNumber("Left Encoder", drive.getRawLeftEncoder());
  }
  
  public void smartdashboard() {
    SmartDashboard.putNumber("Right Encoder", drive.getRawRightEncoder());
    SmartDashboard.putNumber("Left Encoder", drive.getRawLeftEncoder());
  }

  public void gyroStraightTestInit() {
    targX = prefs.getDouble("target X", 0);
    targY = prefs.getDouble("target Y", 0);
    targH = prefs.getDouble("target H", 0);
    DriveStraightConstants.kp = prefs.getDouble("Drive kp", 0);
    DriveStraightConstants.ki = prefs.getDouble("Drive ki", 0);
    DriveStraightConstants.kd = prefs.getDouble("Drive kd", 0);
    GyroStraightConstants.kp = prefs.getDouble("Gyro kp", 0);
    GyroStraightConstants.ki = prefs.getDouble("Gyro ki", 0);
    GyroStraightConstants.kd = prefs.getDouble("Gyro kd", 0);

    SmartDashboard.putNumber("targX", targX);
    SmartDashboard.putNumber("targY", targY);
    SmartDashboard.putNumber("targH", targH);
    SmartDashboard.putNumber("Drive kp", DriveStraightConstants.kp);
    SmartDashboard.putNumber("Drive ki", DriveStraightConstants.ki);
    SmartDashboard.putNumber("Drive kd", DriveStraightConstants.kd);
    SmartDashboard.putNumber("Gyro kp", GyroStraightConstants.kp);
    SmartDashboard.putNumber("Gyro ki", GyroStraightConstants.ki);
    SmartDashboard.putNumber("Gyro kd", GyroStraightConstants.kd);
  }

  public void gyroStraightTest() {
    double distance = (drive.getLeftEncoderDistance() - lastLeftEncoder) + (drive.getRightEncoderDistance() - lastRightEncoder);
    distance /= 2;

    double heading = gyro.getGyroAngle() * Math.PI / 180.0;
    xLoc += distance * Math.cos(heading);
    yLoc += distance * Math.sin(heading);

    SmartDashboard.putNumber("xLoc", xLoc);
    SmartDashboard.putNumber("yLoc", yLoc);

    double deltaX = targX - xLoc;
    double deltaY = targY - yLoc;

    if(oi.driver.getDriverVerticalB1()) {
      SmartDashboard.putBoolean("Is running PID", true);
      new GyroStraightDistancePID(oi.driver.getDriverVertical(), targH, deltaX, deltaY).start();
    } else {
      SmartDashboard.putBoolean("Is running PID", false);
      drive.driveFwdRot(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
    }

    if(oi.driver.getDriverB2()) {
      drive.resetEncoders();
    }
  }

  @Override
  public void disabledInit() {
    super.disabledInit();
    drive.stopMP();
    drive.rawDrive(0, 0);
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
    drive.stopMP();
    drive.rawDrive(0, 0);
  }
  public void testMotors(){
    if(oi.driverVertical.getRawButton(7)){
      drive.motors[0].set(ControlMode.PercentOutput, 0.6);
    }
    else if(oi.driverVertical.getRawButton(8)){
      drive.motors[1].set(ControlMode.PercentOutput, 0.6);
    }
    else if(oi.driverVertical.getRawButton(9)){
      drive.motors[2].set(ControlMode.PercentOutput, 0.6);
    }
    else if(oi.driverVertical.getRawButton(10)){
      drive.motors[3].set(ControlMode.PercentOutput, 0.6);
    }
    else if(oi.driverVertical.getRawButton(11)){
      drive.motors[4].set(ControlMode.PercentOutput, 0.6);
    }
    else if(oi.driverVertical.getRawButton(12)){
      drive.motors[5].set(ControlMode.PercentOutput, 0.6);
    }
    else{
      drive.motors[0].set(ControlMode.PercentOutput, 0.0);
      drive.motors[1].set(ControlMode.PercentOutput, 0.0);
      drive.motors[2].set(ControlMode.PercentOutput, 0.0);
      drive.motors[3].set(ControlMode.PercentOutput, 0.0);
      drive.motors[4].set(ControlMode.PercentOutput, 0.0);
      drive.motors[5].set(ControlMode.PercentOutput, 0.0);

    }
  }
}

enum SubsystemEnum {
  AllEssentials, Drive, Camera, Gyro, LED, Lift, Vision, Arm, CargoDeploy;
}