/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.CargoDeploy;
import frc.robot.commands.DrivetrainTest;
import frc.robot.commands.RunCargoDeployCommand;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CargoDeploySubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.LedSubsystem;
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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static OI oi;
  public static CargoDeploySubsystem cargoDeploy;
  public static DrivetrainSubsystem drive;
  public static LedSubsystem led;
  public static CameraSubsystem camera;
  public static GyroSubsystem gyro;

  public static int resWidth;
  public static int resHeight;

  public static Preferences prefs;

  public static int timeoutMs = 20;

   public static TalonSRX motor1;
   public static TalonSRX motor2;
   Joystick joy = new Joystick(0);
 

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
<<<<<<< HEAD
    //  motor1 = new TalonSRX(15);
    //  motor2 = new TalonSRX(16);

=======
    // motor1 = new TalonSRX(0);
    // motor2 = new TalonSRX(0);
    Shuffleboard.getTab("Drive").add("Time Left", Timer.getFPGATimestamp()).withSize(2, 4).withPosition(2,4)
                        .withWidget(BuiltInWidgets.kNumberBar).getEntry();
>>>>>>> 5a288e2772560c8f6c64ad081dc80d9c9bb3129e
    


    oi = new OI();
    drive = new DrivetrainSubsystem();
    cargoDeploy = new CargoDeploySubsystem();
    led = new LedSubsystem();
    camera = new CameraSubsystem();
    gyro = new GyroSubsystem();

    prefs = Preferences.getInstance();
<<<<<<< HEAD


 
  
=======
>>>>>>> 5a288e2772560c8f6c64ad081dc80d9c9bb3129e
  }
   /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("bandwidth", camera.max);
    // System.out.println(camera.max);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    // Drivetrain Testing Commands
    //new DrivetrainTest().start();
    //new DriveTest().start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // FOR CONNOR
<<<<<<< HEAD
    // new RunCargoDeployCommand().start();

    
    // motor1.set(ControlMode.PercentOutput, joy.getY());
    // motor2.set(ControlMode.PercentOutput, joy.getY());
    
    int motorNumber = prefs.getInt("MotorNumber", 0);
    SmartDashboard.putNumber("MotorNumber", motorNumber);
    drive.motors[5].set(ControlMode.PercentOutput, oi.driver.getDriverVertical());
    //SmartDashboard.putNumber("bandwidth", camera.max);
    
    smartdashboard();
=======
    // new RunCargoDeployCommand().start
    // motor1.set(ControlMode.PercentOutput, .5);
    // motor2.set(ControlMode.PercentOutput, .5);
    int motorNumber = prefs.getInt("MotorNumber", 0);

    drive.motors[motorNumber].set(ControlMode.PercentOutput, oi.driver.getDriverVertical());
    //SmartDashboard.putNumber("bandwidth", camera.max);
>>>>>>> 5a288e2772560c8f6c64ad081dc80d9c9bb3129e
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void smartdashboard() {
    SmartDashboard.putNumber("Right Encoder", drive.getRightEncoder());
    SmartDashboard.putNumber("Left Encoder", drive.getLeftEncoder());
  }
}