/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotMap.CargoDeploy;
import frc.robot.RobotMap.Jacks;
import frc.robot.commands.GyroStraightDistancePID;
import frc.robot.commands.test.*;
import frc.robot.commands.JackMotionProfileCommand;
import frc.robot.commands.RunCargoDeployCommand;
import frc.robot.commands.VisionFixCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.JackMotionProfileAndLiftCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.commands.JackMotionProfileCommand;


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
  public static LiftSubsystem lift;
  public static JackSubsystem jack;

  // Vision
  public static VisionSubsystem vision;
  public static VisionFixCommand visionFixCommand;

  // public final static Map<SubsystemEnum, Subsystem> loggingArr = new HashMap<>() {{
  //     put(SubsystemEnum.Drive, drive);
  //     put(SubsystemEnum.CargoDeploy, cargoDeploy);
  //     put(SubsystemEnum.LED, led);
  //     put(SubsystemEnum.Camera, camera);
  //     put(SubsystemEnum.Gyro, gyro);
  //     put(SubsystemEnum.Vision, vision);
  // }};

  public static Preferences prefs;

  public static int timeoutMs = 20;

  public static TalonSRX motor1;
  public static TalonSRX motor2;
  Timer accelTime;
  Joystick joy1 = new Joystick(0);
  Joystick joy2 = new Joystick(1);

  public double prev_vel = 0.0; // TODO put in diagnostic

  public double max_vel = 0.0;
  public double max_accel = 0.0;
  public double lastLeftEncoder;
  public double lastRightEncoder;
  public double xLoc;
  public double yLoc;
  public double firstTargX;
  public double firstTargY;
  public double targX;
  public double targY;
  public double targH;
  public boolean onFirstLeg;

 
  public double initAccel;

  public static JackMotionProfileCommand jackMpCommand;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    Shuffleboard.getTab("Drive").add("Time Left", Timer.getFPGATimestamp()).withSize(2, 4).withPosition(2,4)
                        .withWidget(BuiltInWidgets.kNumberBar).getEntry();
    //******************************************** Subsystems */
    oi = new OI();
    drive = new DrivetrainSubsystem();
    cargoDeploy = new CargoDeploySubsystem();
    // cargoIntake = new CargoIntakeSubsystem();
    led = new LedSubsystem();
    jack = new JackSubsystem();
    //camera = new CameraSubsystem();
    gyro = new GyroSubsystem();
    lift = new LiftSubsystem();
    try {
      vision = new VisionSubsystem();
    }
    catch(Exception e){
      SmartDashboard.putString("VISION INIT FAILED", "1");
      e.printStackTrace();
    }
    prefs = Preferences.getInstance();
    initAccel = 0;
    drive.changeBrakeCoast(false);
    accelTime = new Timer();
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
    jack.resetJackEncoder();
    // jackMpCommand = new JackMotionProfileCommand(6000,true,4.0);
    // jackMpCommand.start();
    // lift.setLiftMPHang(); 
    // jack.setJackMPVals(true);  
    // (new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE, true, 10.0)).start();
    drive.resetEncoders();
    drive.runPath("TestPath", 1.0, 0, 0.0, 16.0, 0.01);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    // lift.pidLift(100);
    // new JackMotionProfileCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE, true, 10.0).start();
    SmartDashboard.putNumber("Jack Encoder", jack.getJackEncoder());
    // lift.motionMagicLift(LiftSubsystem.LiftEncoderConstants.INTAKE);
  }

  @Override
  public void teleopInit() {
    drive.stopMP();
    gyro.resetGyro();
    jack.resetJackEncoder();
    visionFixCommand = new VisionFixCommand();
    drive.changeBrakeCoast(false);
    lift.resetEncoder();
    // gyroStraightTestInit();
    // drive.stopMP();
    // drive.stopMP();
    // gyro.resetGyro();
    // visionFixCommand = new VisionFixCommand();
    // drive.changeBrakeCoast(false);
    // new LiftTest().start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // gyroStraightTest();

    testJacks();

    // NOTE: THE VISION FIX COMMAND OVVERRIDES THE STANDARD TELEOP ARCADE DRIVING.
    // if(oi.visionFixButton.get()){
    //   visionFixCommand.start();
    //   return;

    // }
    // else {
    //   visionFixCommand.cancel();
    //   drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
    // }
    lift.setManualLift(oi.driver.getDriverVertical());
    SmartDashboard.putNumber("Lift Encoder", lift.getRawEncoder());

    // int motorNumber = prefs.getInt("MotorNumber", 0);
    // sendShuffleboard(new SubsystemEnum[] {SubsystemEnum.AllEssentials});
    // SmartDashboard.putNumber("left e", drive.getRawLeftEncoder());
    // SmartDashboard.putNumber("right e", drive.getRawRightEncoder());


    // if (oi.driver.getDriverButton1()) {
    //   drive.resetEncoders();
    // }
   
  }

  @Override
  public void testInit() {
    this.gyro.resetGyro();
    this.drive.resetEncoders();
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

  
  // public void sendShuffleboard(SubsystemEnum[] subs) {
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
    // SmartDashboard.putNumber("Right Encoder", drive.getRightEncoder());
    // SmartDashboard.putNumber("Left Encoder", drive.getLeftEncoder());
  // }

  public void gyroStraightTestInit() {
    lastLeftEncoder = 0;
    lastRightEncoder = 0;
    xLoc = 0;
    yLoc = 0;
    onFirstLeg = true;
    drive.resetEncoders();

    targX = prefs.getDouble("target X", 0);
    targY = prefs.getDouble("target Y", 0);
    targH = prefs.getDouble("Angle", 0);
    DrivetrainSubsystem.DriveStraightGyroConstants.kp = prefs.getDouble("Drive Gyro kp", 0);
    DrivetrainSubsystem.DriveStraightGyroConstants.ki = prefs.getDouble("Drive Gyro ki", 0);
    DrivetrainSubsystem.DriveStraightGyroConstants.kd = prefs.getDouble("Drive Gyro kd", 0);
    GyroSubsystem.GyroStraightConstants.kp = prefs.getDouble("Gyro kp", 0);
    GyroSubsystem.GyroStraightConstants.ki = prefs.getDouble("Gyro ki", 0);
    GyroSubsystem.GyroStraightConstants.kd = prefs.getDouble("Gyro kd", 0);
    DrivetrainSubsystem.DriveStraightConstants.kp = prefs.getDouble("Drive kp", 0);
    DrivetrainSubsystem.DriveStraightConstants.ki = prefs.getDouble("Drive ki", 0);
    DrivetrainSubsystem.DriveStraightConstants.kd = prefs.getDouble("Drive kd", 0);
    GyroSubsystem.GyroStraightConstants.kp = prefs.getDouble("Gyro kp", 0);
    GyroSubsystem.GyroStraightConstants.ki = prefs.getDouble("Gyro ki", 0);
    GyroSubsystem.GyroStraightConstants.kd = prefs.getDouble("Gyro kd", 0);

    SmartDashboard.putNumber("targX", targX);
    SmartDashboard.putNumber("targY", targY);
    SmartDashboard.putNumber("targH", targH);
    SmartDashboard.putNumber("Drive kp", DrivetrainSubsystem.DriveStraightConstants.kp);
    SmartDashboard.putNumber("Drive ki", DrivetrainSubsystem.DriveStraightConstants.ki);
    SmartDashboard.putNumber("Drive kd", DrivetrainSubsystem.DriveStraightConstants.kd);
    SmartDashboard.putNumber("Gyro kp", GyroSubsystem.GyroStraightConstants.kp);
    SmartDashboard.putNumber("Gyro ki", GyroSubsystem.GyroStraightConstants.ki);
    SmartDashboard.putNumber("Gyro kd", GyroSubsystem.GyroStraightConstants.kd);

    firstTargX = targX/2.0;
    firstTargY = targH/2.0;
  }

  @Override
  public void disabledInit() {
    // Remove all commands from queue
    Scheduler.getInstance().removeAll();

    // Stop drivetrain motion
    drive.stopMP();
    drive.rawDrive(0, 0);

    // Stop lift motion

    // Stop jack motion - note: robot safety is priority. Is it safe for the robot's jack to stop running?
  }

  @Override
  public void disabledPeriodic() {
    
  }
  public void testJacks(){
    // SmartDashboard.putNumber("joy y", oi.operator.getY());
    jack.setJackMotor(oi.operator.getY());
    if(oi.operator.getButton7()){
      jack.setJackDriveMotor(1.0);
    }else if(oi.operator.getButton8()){
      jack.setJackDriveMotor(-1.0);
    }else{
      jack.setJackDriveMotor(0.0);
    }
    SmartDashboard.putBoolean("Is Jack Down", jack.isJackAtBottom());
    SmartDashboard.putNumber("Jack Encoder", jack.getJackEncoder());
    SmartDashboard.putNumber("Jack Vel", jack.getJackVel());
    if(accelTime.get() == 0.0){
      initAccel = jack.getJackVel();
      accelTime.start();
    }else if(accelTime.get()>0.1){
      accelTime.stop();
      SmartDashboard.putNumber("Jack Accel", (jack.getJackVel() - initAccel) /(accelTime.get()/0.1));
      accelTime.reset();
    }

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
    }else{
      drive.motors[0].set(ControlMode.PercentOutput, 0.0);
      drive.motors[1].set(ControlMode.PercentOutput, 0.0);
      drive.motors[2].set(ControlMode.PercentOutput, 0.0);
      drive.motors[3].set(ControlMode.PercentOutput, 0.0);
      drive.motors[4].set(ControlMode.PercentOutput, 0.0);
      drive.motors[5].set(ControlMode.PercentOutput, 0.0);

    }

    SmartDashboard.putNumber("Heading", gyro.getGyroAngle());
    SmartDashboard.putNumber("Right Encoder", drive.getRightEncoderDistance());
    SmartDashboard.putNumber("Left Encoder", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Left Encoder Raw", drive.getRawLeftEncoder());
    SmartDashboard.putNumber("Right Encoder Raw", drive.getRawRightEncoder());
    // SmartDashboard.putNumber("Delta X", deltaX);
    // SmartDashboard.putNumber("Delta Y", deltaY);
    SmartDashboard.putBoolean("On First Leg", onFirstLeg);
  }
}
