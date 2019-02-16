/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.*;
import frc.robot.RobotMap.Drivetrain;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.commands.test.IntakeMotorsTest;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.subsystems.LiftSubsystem.PIDConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  public static LiftSubsystem lift;
  public static JackSubsystem jack;
  public static HatchSubsystem hatch;

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

  public static Compressor compressor;

  public static int timeoutMs = 20;

  public static TalonSRX motor1;
  public static TalonSRX motor2;
  Timer time;
  Timer intakeTime;
  Timer accelTime;
  Joystick joy1 = new Joystick(0);
  Joystick joy2 = new Joystick(1);

  // DRIVE kinematics tracking
  public double prev_vel = 0.0; // TODO put in diagnostic
  public double prev_time = 0.0;
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
  public double initAccel = 0.0;

  
  public int maxLiftAccel;
  public int maxLiftVel;
  public int lastLiftVel;
 
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
    cargoIntake = new CargoIntakeSubsystem();
    led = new LedSubsystem();
    jack = new JackSubsystem();
    camera = new CameraSubsystem();
    gyro = new GyroSubsystem();
    lift = new LiftSubsystem();
    // hatch = new HatchSubsystem();
    try {
      //vision = new VisionSubsystem();
    }
    catch(Exception e){
      SmartDashboard.putString("VISION INIT FAILED", "1");
      e.printStackTrace();
    }
    prefs = Preferences.getInstance();
    drive.changeBrakeCoast(false);
    accelTime = new Timer();
    // compressor = new Compressor(4);
    time = new Timer();
    accelTime.start();
    prev_time = accelTime.get();
  }
   /**
   * This function is ca
   * lled every robot packet, no matter the mode. Use
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

    // kinematics negated b/c robot tips while driving fwd without most of robot weight.
    SmartDashboard.putNumber("GYRO HEADING", gyro.getGyroAngle());
    SmartDashboard.putNumber("V", -drive.leftEncoder.getRate());    
    SmartDashboard.putNumber("MAX V", max_vel = Math.max(max_vel, -(drive.leftEncoder.getRate())));  
    // only calculate avg A over 0.5 seconds because instantaneous stuff is way off (? why ?)
    if(prev_time + 0.5 <= accelTime.get()) {
      double curA = ((drive.leftEncoder.getRate()) - prev_vel)/(accelTime.get() - prev_time);
      SmartDashboard.putNumber("A", curA);
      SmartDashboard.putNumber("MAX A", max_accel = Math.max(max_accel, -curA));
      prev_vel = -(drive.leftEncoder.getRate());
      prev_time = accelTime.get();
      SmartDashboard.putNumber("PREV TIME", prev_time);
    }  
    SmartDashboard.putNumber("LEFT ENCODER", drive.leftEncoder.get());
    SmartDashboard.putNumber("RIGHT ENCODER", drive.rightEncoder.get());
    SmartDashboard.putNumber("TIME", accelTime.get());

    SmartDashboard.putNumber("SAMPLE RATE", drive.leftEncoder.getSamplesToAverage());
    SmartDashboard.putNumber("SAMPLE RATE R", drive.rightEncoder.getSamplesToAverage());


    // vision.readData();
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
    gyro.resetGyro();
    // drive.runPath("HAB1L-ROCKLF-1", 0.18, 0.0, 0.02, 16.0, 0.008, true);
    // drive.runPath("test-HABLI-CLF", 0.18, 0.0, 0.02, 16.0, 0.008, false);

    // new HAB1LxCLFxLOADLxCL1().start();
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
    // jack.resetJackEncoder();
    gyro.resetGyro();
    // visionFixCommand = new VisionFixCommand();
    drive.changeBrakeCoast(false);

    // getInitLiftValues();
    // new IntakeMotorsTest().start();
    // lift.resetEncoder();
    // gyroStraightTestInit();
    // drive.stopMP();
    // drive.stopMP();
    // gyro.resetGyro();
    // visionFixCommand = new VisionFixCommand();
    // drive.changeBrakeCoast(false);
    // new LiftTest().start();

    // compressor.setClosedLoopControl(true);
    // compressor.start();
    intakeTime = new Timer();
    intakeTime.reset();
    intakeTime.start();
    // cargoIntake.isOut = false;
    lift.resetEncoder();
    initLiftTune();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // gyroStraightTest();
    SmartDashboard.putBoolean("LeftSol", cargoIntake.getLeftSol());
    SmartDashboard.putBoolean("RightSol", cargoIntake.getRightSol());
    SmartDashboard.putBoolean("IsOut", cargoIntake.solOut());
    // testJacks();
    // if(oi.gamePad.getButtonA() && intakeTime.get() > 0.5) {
    //   cargoIntake.switchSol();
    //   intakeTime.reset();
    // }
    // if(oi.gamePad.getRightButton()) { //extake
    //   cargoIntake.runIntake(true, 1);
    //   cargoDeploy.runIntake(1);
    // } else if(oi.gamePad.getLeftButton()) { //intake
    //   cargoIntake.runIntake(false, 1);
    //   cargoDeploy.runIntake(-1);
    // } else {
    //   cargoIntake.runIntake(true, 0);
    //   cargoDeploy.runIntake(0);
    // }


    liftTune();
    // drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
    // getLiftValues();
    // lift.setManualLift(oi.driver.getDriverVertical());
    // NOTE: THE VISION FIX COMMAND OVVERRIDES THE STANDARD TELEOP ARCADE DRIVING.
    // if(oi.visionFixButton.get()){
    //   visionFixCommand.start();
    //   return;

    // }
    // else {
    //   visionFixCommand.cancel();
    //   drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
    // }
    // lift.setManualLift(oi.driver.getDriverVertical());
    // SmartDashboard.putNumber("Lift Encoder", lift.getRawEncoder());

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
    gyro.resetGyro();
    drive.resetEncoders();
    Command c = (new MotionProfileCommand("TestPath", false));
    c.start();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();

    // TEST GYRO PID
    // gyro.gyroPIDController.enable();
    // double pidOut = gyro.getGyroPIDOutput();
    // drive.rawDrive(pidOut, -pidOut);

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
    // Scheduler.getInstance().removeAll();

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

  public void getInitLiftValues() {
    maxLiftAccel = 0;
    maxLiftVel = 0;
    time = new Timer();
    time.reset();
    time.start();
  }

  public void getLiftValues() {
    // int currentAccel = ;
    int currentVel = lift.getVel();
    if (Math.abs(currentVel) > maxLiftVel) {
      maxLiftVel = Math.abs(currentVel);
    }
    double sec = time.get();
    int accel = (int)(((double)(currentVel - lastLiftVel))/sec);
    if(Math.abs(accel) > maxLiftAccel) {
      maxLiftAccel = Math.abs(accel);
    }
    lastLiftVel = currentVel;
    time.reset();
    SmartDashboard.putNumber("MaxAccel", maxLiftAccel);
    SmartDashboard.putNumber("MaxVel", maxLiftVel);
    SmartDashboard.putNumber("Curr Vel", currentVel);
    SmartDashboard.putNumber("Curr Accel", accel);
    SmartDashboard.putNumber("Position", lift.getRawEncoder());
    }

    public void initLiftTune() {
      LiftSubsystem.PIDConstants.k_p = prefs.getDouble("k-p", 0.0);
      LiftSubsystem.PIDConstants.k_i = prefs.getDouble("k-i", 0.0);
      LiftSubsystem.PIDConstants.k_d = prefs.getDouble("k-d", 0.0);

      SmartDashboard.putNumber("k-p", PIDConstants.k_p);
      SmartDashboard.putNumber("k-i", PIDConstants.k_i);
      SmartDashboard.putNumber("k-d", PIDConstants.k_d);

      lift.setLiftPID();
    }

    public void liftTune() {
      if(oi.driver.getDriverButton1()) {
        lift.motionMagicLift(4466);
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.driver.getDriverButton3()) {
        lift.motionMagicLift(8000);
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.driver.getDriverButton4()) {
        new MoveLiftCommand(LiftPositions.HATCH_MID).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(!lift.getIsMoving()) {
        lift.setManualLift(oi.driver.getDriverVertical());
        SmartDashboard.putBoolean("Is Motion Magic", false);
      }
      if(oi.driver.getDriverButton2()) {
        lift.resetEncoder();
        lift.setPosition(LiftPositions.LOW);
      }
      SmartDashboard.putNumber("Lift Pos", lift.getRawEncoder());
    }
}
