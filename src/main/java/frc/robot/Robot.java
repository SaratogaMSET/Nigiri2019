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
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.intake.RunCargoIntake;
import frc.robot.commands.intake.SetIntakePistons;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.commands.intake.SetMidStatePistons;
import frc.robot.commands.test.IntakeMotorsTest;
import frc.robot.commands.test.LiftTest;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakeState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.subsystems.LiftSubsystem.PIDConstants;
import frc.robot.util.RobotState;

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
  public static RobotState robotState;
  public static boolean isGamepad = true;

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

  public static Preferences prefs;

  public static Compressor compressor;

  public static int timeoutMs = 20;


  // DRIVE kinematics tracking
  public double prev_vel = 0.0; // TODO put in diagnostic
  public double prev_time = 0.0;
  public double max_vel = 0.0;
  public double max_accel = 0.0;
  public Timer accelTime = new Timer();

  public Timer intakeTime, time;
 
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
    hatch = new HatchSubsystem();
    compressor = new Compressor(4);
    prefs = Preferences.getInstance();
    drive.changeBrakeCoast(false);

    try {
      vision = new VisionSubsystem();
    }
    catch(Exception e){
      SmartDashboard.putString("VISION INIT FAILED", "1");
      e.printStackTrace();
    }
    prefs = Preferences.getInstance();
    robotState = new RobotState();

    drive.changeBrakeCoast(false);
  }
   /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specififc periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating. </p>
   * 
   * 649: Because of ^ that fact, we can use this method to run our safety checks and override any dangerous behavior our mode-specific loops actuate.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("bandwidth", camera.max);
    // System.out.println(camera.max);
    double vel = (drive.leftEncoder.getRate() + drive.rightEncoder.getRate())/2.0f;
    SmartDashboard.putNumber("GYRO HEADING", gyro.getGyroAngle());
    SmartDashboard.putNumber("V", vel);
    SmartDashboard.putNumber("MAX V", max_vel = Math.max(max_vel, vel));  
    // only calculate avg A over 0.5 seconds because instantaneous stuff is way off (? why ?)
    if(prev_time + 0.5 <= accelTime.get()) {
      double curA = (vel - prev_vel)/(accelTime.get() - prev_time);
      SmartDashboard.putNumber("A", curA);
      SmartDashboard.putNumber("MAX A", max_accel = Math.max(max_accel, -curA));
      prev_vel = vel;
      prev_time = accelTime.get();
      SmartDashboard.putNumber("PREV TIME", prev_time);
    }  
    SmartDashboard.putNumber("LEFT ENCODER", drive.leftEncoder.get());
    SmartDashboard.putNumber("RIGHT ENCODER", drive.rightEncoder.get());
    // SmartDashboard.putNumber("TIME", accelTime.get());

    // SmartDashboard.putNumber("SAMPLE RATE", drive.leftEncoder.getSamplesToAverage());
    // SmartDashboard.putNumber("SAMPLE RATE R", drive.rightEncoder.getSamplesToAverage());
    // SmartDashboard.putNumber("JACK ENCODER", jack.getJackEncoder());
    // SmartDashboard.putNumber("LIFT ENCODER", lift.getRawEncoder());
    // SmartDashboard.putBoolean("CARGO IR SENSOR", cargoDeploy.hasCargo());
    // SmartDashboard.putBoolean("JACK UP HAL", jack.isJackAtTop());
    smartdashboardTesting();

    // Safety Checks
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
    drive.resetEncoders();
    gyro.resetGyro();
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
    Robot.drive.rawDrive(0.0, 0.0);
    Robot.lift.resetEncoder();
    gyro.resetGyro();
    drive.changeBrakeCoast(false);

    compressor.setClosedLoopControl(true);
    compressor.start();

    SmartDashboard.putNumber("Low Rocket", lift.getTicksFromDistance(LiftSubsystem.LiftDistanceConstants.CARGO_ROCKET_LEVEL_ONE));
    SmartDashboard.putNumber("Mid Rocket", lift.getTicksFromDistance(LiftSubsystem.LiftDistanceConstants.CARGO_ROCKET_LEVEL_TWO));
    // new LiftTest().start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    teleopLoop();
    
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

  

  @Override
  public void disabledInit() {
    // Remove all commands from queue
    Scheduler.getInstance().removeAll();

    // Stop drivetrain motion
    drive.rawDrive(0, 0);

    // Stop lift motion

    // Stop jack motion - note: robot safety is priority. Is it safe for the robot's jack to stop running?
  }

  @Override
  public void disabledPeriodic() {
    
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
  }

  public void teleopLoop() {
    //******************************* INTAKE ***********************************************/
    if(oi.gamePad.getLeftButtonPressed()) {
      new ChangeIntakeState(CargoIntakeState.OUT).start();
      new SetIntakeRollers(true, 0.75).start();
    } else if(oi.gamePad.getLeftButtonReleased()) {
      SmartDashboard.putBoolean("Intake Pressed", false);
      new ChangeIntakeState(CargoIntakeState.MID).start();
      new SetIntakeRollers(false, 0).start();
    }

    //****************************** LIFTING *************************************************/
    if(oi.gamePad.getButtonAPressed()) { // ****************************** LIFT TO LOW
      new MoveLiftCommand(LiftPositions.LOW, 2).start();
    } 
    if(oi.gamePad.getButtonXPressed()) { // *********************** LIFT TO HATCH MID
      new MoveLiftCommand(LiftPositions.HATCH_MID, 2).start();
    }
    if(oi.gamePad.getButtonYPressed()) { // *********************** LIFT TO HATCH HIGH
      new MoveLiftCommand(LiftPositions.HATCH_HIGH, 2).start();
    }
    if(oi.gamePad.getButtonXPressed() && oi.gamePad.getLeftTrigger()) { // **** LIFT TO LOW ROCKET
      new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_ONE, 2).start();
    }
    if(oi.gamePad.getButtonYPressed() && oi.gamePad.getLeftTrigger()) { // **** LIFT TO MID ROCKET
      new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_TWO, 2).start();
    }
    if(oi.gamePad.getButtonBPressed() && oi.gamePad.getLeftTrigger()) { // **** LIFT TO HIGH ROCKET
      new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_THREE, 2).start();
    }
    if(oi.gamePad.getRightButtonPressed()) { // ******************* LIFT TO CARGO SHIP
      new MoveLiftCommand(LiftPositions.CARGO_SHIP, 2).start();
    }
    if(oi.gamePad.getRightTrigger() && oi.gamePad.getLeftTrigger()) { // ****** LIFT LOADING STATION
      new MoveLiftCommand(LiftPositions.CARGO_LOADING_STATION, 2).start();
    }


    // *************************** DEPLOY **********************************************/
    if(oi.gamePad.getBackButtonPressed()) { 
      new SetIntakeRollers(false, 0.75).start();
      cargoDeploy.runIntake(0.75);
      hatch.hatchDeploy();
    } else if(oi.gamePad.getBackButtonReleased()) {
      new SetIntakeRollers(false, 0).start();
      hatch.hatchDeployIn();
    }

    // if(oi.driver.driverDeploy()) {
    //   new SetIntakeRollers(false, 0.75).start();
    //   hatch.hatchDeploy();
    // } else {
    //   new SetIntakeRollers(false, 0).start();
    //   hatch.hatchDeployIn();
    // }

    // ****************************** JACK ***********************************************/
    if(oi.gamePad.getPOVLeft()) {
      // start climb sequence level 2
    } else if(oi.gamePad.getPOVRight()) {
      // start climb sequence level 3
    } else if(oi.gamePad.getPOVDown()) {
      // push down jack
    } else if(oi.gamePad.getPOVUp()) {
      // push up jack
    }

    if(oi.gamePad.getStartButton()) { // ****************** MANUAL MODE
      double pow = oi.gamePad.getLeftJoystickY()/2;
      if(lift.getBottomHal() && pow < 0){
        pow = 0;
      }
      lift.setManualLift(pow);
    }

    //******************************* DRIVE ****************************************/
    drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
  }

  public void smartdashboardTesting() {
    //***************************************************** DRIVE */
    SmartDashboard.putNumber("Left Encoder Raw", drive.getRawLeftEncoder());
    SmartDashboard.putNumber("Left Encoder Distance", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Raw", drive.getRawRightEncoder());
    SmartDashboard.putNumber("Right Encoder Raw", drive.getRightEncoderDistance());

    //***************************************************** LIFT */
    SmartDashboard.putNumber("Lift Encoder Raw", lift.getRawEncoder());
    SmartDashboard.putNumber("Lift Distance", lift.getDistance());
    SmartDashboard.putBoolean("Bottom Hal", lift.getBottomHal());

    //***************************************************** INTAKE */
    SmartDashboard.putBoolean("Up/Down Sol State", cargoIntake.getIntakeSolState());
    SmartDashboard.putBoolean("Mid State Sol State", cargoIntake.getMidStateSolState());
    SmartDashboard.putBoolean("In Hal", cargoIntake.getInHal());
    SmartDashboard.putBoolean("Out Hal", cargoIntake.getOutHal());

    //***************************************************** JACK */
    SmartDashboard.putBoolean("Jack Deployed Hal", jack.isJackAtTop());
    SmartDashboard.putBoolean("Jack Stored Hal", jack.isJackAtBottom());
    SmartDashboard.putNumber("Jack Encoder", jack.getJackEncoder());

    //***************************************************** HATCH */


    //***************************************************** VISION */


    //***************************************************** CURRENT ROBOT STATES */
    SmartDashboard.putString("Lift State", RobotState.liftPosition.toString());
    SmartDashboard.putString("Hatch State", RobotState.hatchState.toString());
    SmartDashboard.putString("Intake State", RobotState.cargoIntakeState.toString());

    // SmartDashboard.putString("Currently Running Command", Scheduler.getInstance().getName());
  }
}
