/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.*;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.commands.intake.WaitUntilLiftDownIntake;
import frc.robot.commands.semiauto.DefenseModeCommand;
import frc.robot.commands.semiauto.climb.DeployClimbForks;
import frc.robot.commands.semiauto.climb.JackMotionProfileAndLiftCommand;
import frc.robot.commands.semiauto.climb.PrepareClimb2;
import frc.robot.commands.semiauto.climb.PrepareClimb3;
import frc.robot.commands.semiauto.climb.TestJackDriveMotors;
import frc.robot.commands.test.IntakeMotorsTest;
import frc.robot.commands.test.LiftTest;
import frc.robot.commands.test.TestDTMaxVA;
import frc.robot.commands.test.TestTalonVelocity;
import frc.robot.commands.vision.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.util.FishyMath;
import frc.robot.util.Logging;
import frc.robot.util.RobotState;
import jaci.pathfinder.Pathfinder;
// import sun.util.logging.PlatformLogger.Level;
import frc.robot.commands.semiauto.climb.MoveJackCommand;


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

  public static boolean isDefenseMode = false;
  public static boolean autoDriverControl = false;
  public static boolean isManualMode = false;
  public static boolean isLogging = true;


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
  public static AutoSelector autoSelector;
  private static Subsystem[] subsystems = {
    cargoDeploy, drive, led, camera, gyro, cargoIntake, lift, jack, hatch, autoSelector
  };

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

  public static Timer intakeTime, time;

  public static boolean isClimb;
  public static boolean isLevel3;
  public static boolean isJackRunning;

  public Command autoCommand;
 
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
    // autoSelector = new AutoSelector();
    camera = new CameraSubsystem();
    gyro = new GyroSubsystem();
    lift = new LiftSubsystem();
    hatch = new HatchSubsystem();
    compressor = new Compressor(4);
    prefs = Preferences.getInstance();
    drive.changeBrakeCoast(false);
   
    try {
      vision = new VisionSubsystem();
      SmartDashboard.putString("VISION INIT", "1");
    } catch(Exception e){
      SmartDashboard.putString("VISION INIT", "0");
      e.printStackTrace();
    }
    prefs = Preferences.getInstance();
    robotState = new RobotState();

    drive.changeBrakeCoast(false);
    lift.resetEncoder();
    jack.resetJackEncoder();

    time = new Timer();


    led.solidRed();

    autoCommand = new MotionProfileCommand("FarRocketLeft", true);
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
    //***********************************UPDATE STATES***************************** */
    cargoIntake.updateIntakeState();
    cargoIntake.updateIntakeRollerState();
    cargoDeploy.updateCargoGamePieceState();
    cargoDeploy.updateCargoDeployState();
    hatch.updateHatchPositionState();
    hatch.updateHatchDeployState();
    RobotState.liftPosition = lift.updateLiftPosition();
    smartdashboardTesting();

    //*********************************CHECK STATE VALIDITY*********************** */
    cargoIntake.checkIntakeState();
    hatch.checkHatchStateValid();

    SmartDashboard.putBoolean("Is Defense Mode", isDefenseMode);

    // SmartDashboard.putString("Auto", autoSelector.getAuto());
    // SmartDashboard.putString("Side", autoSelector.getSide());
    // SmartDashboard.putString("Control", autoSelector.getControl());

    // Safety Checks
    if(!jack.isJackAtTop() && !isClimb){
      jack.setJackMotorMP(JackSubsystem.JackEncoderConstants.UP_STATE);
    }
    if(jack.isJackAtTop()) {
      jack.resetJackEncoder();
    }

    // Vision
    if(vision != null) {
      if(visionFixCommand != null && !visionFixCommand.isRunning()) {
        vision.readData();
      }

      Double angle = vision.getAngleDisplacement();
      if(angle != null) {
        if(Math.abs(angle) < 3.0) {
          led.solidGreen();
          SmartDashboard.putBoolean("VSTATUS", true);
        } else {
          led.solidBlue();
          SmartDashboard.putBoolean("VSTATUS", false);
        }
      } else {
        led.solidRed();
      }
    } else {
      led.solidRed();
    }

    if(lift.getBottomHal()){
      lift.resetEncoder();
    }
  }

   /* LabVIEW Dashboard, remove all of the chooser code and uncomment the
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
    // (new MotionProfileCommand("FarRocketLeft1", true)).start();
    // new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE_2, true, 30.0).start();
    // new TestJackDriveMotors().start();
    // new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE_LEVEL_3, true, 30.0).start();
    // new TestDTMaxVA(10.0).start();
    // new HAB1LxROCKLF().start();
    // autoCommand.start();
    // new TestTalonVelocity(100).start();
    new PrepareClimb2().start();


  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    // Auto-Teleop Shifting (Using Trigger)
    if(oi.driver.getDriverButton1()) {
      autoDriverControl = !autoDriverControl;
      init(autoDriverControl);
    }
    if(autoDriverControl) {
      teleopLoop();
    } else {
      if(oi.driver.getDriverButton2() || oi.driver.getDriverButton3()) {
        // Kill switch
        Scheduler.getInstance().removeAll();
        stopAll();
      }
      // Put all the auto code here–the stuff you normally run in auto
      // lift.pidLift(100);
      // new JackMotionProfileCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE,
      // true, 10.0).start();
      // lift.motionMagicLift(LiftSubsystem.LiftEncoderConstants.INTAKE);
      SmartDashboard.putNumber("Jack Encoder", jack.getJackEncoder());
    }
  }

  public void stopAll() {
    for(Subsystem s : subsystems) {s.stopAll();}
  }

  @Override
  public void teleopInit() {
    init(false);
    time.reset();
    time.start();
    if(isLogging) {
      Logging.createLogFile();
    }
    isClimb = false; 
    isLevel3 = false;
    isJackRunning = false;
  }

  //Init method for both auto and teleop
  public void init(boolean auto) {
    if(auto) {
      drive.resetEncoders();
      gyro.resetGyro();
      // new HAB1LxCLFxLOADLxCL1().start();
      // (new MotionProfileCommand("FarRocketLeft1", true)).start();
      // new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE_2, true, 30.0).start();
      // new TestJackDriveMotors().start();
      // new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstatns.DOWN_STATE_LEVEL_3, true, 30.0).start();
      // new TestDTMaxVA(10.0).start();
      new HAB1LxROCKLF().start();
    } else {
      Robot.drive.rawDrive(0.0, 0.0);
      gyro.resetGyro();
      drive.changeBrakeCoast(false);
      visionFixCommand = new VisionFixCommand();

      compressor.setClosedLoopControl(true);
      compressor.start();

      // new WaitUntilEncoderCommand(2, new LedPatternCommand(3, 5), 30).start();
    }
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
    // gyro.gyroPIDController.setSetpoint(90.0);
    // gyro.gyroPIDController.enable();
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();

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
    // hatch.changeHatchState();
    // Stop lift motion

    // Stop jack motion - note: robot safety is priority. Is it safe for the robot's jack to stop running?
    if(isLogging) {
      Logging.closeWriter();
    }
  }

  @Override
  public void disabledPeriodic() {
    
  }

  public void teleopLoop() {
    //******************************* INTAKE ***********************************************/
    if(!isClimb && !isDefenseMode){
      if(oi.gamePad.getLeftButtonPressed()) { //****************RUN INTAKES*********** */
        if(RobotState.liftPosition != LiftPositions.LOW
          && !lift.isMoving())  {
          new SetIntakeRollers(true, 0, 0, 1).start(); // only carriage
        } else {
          new ChangeIntakeState(CargoIntakePositionState.OUT).start();
          new WaitUntilLiftDownIntake(true, 1, 1, 1, 3).start();
        }
        compressor.stop();
      } else if(oi.gamePad.getLeftButtonReleased()) { // *******STOP AND STORE********* */
        new ChangeIntakeState(CargoIntakePositionState.MID).start();
        new SetIntakeRollers(false, 0, 0, 0).start();
        compressor.start();
      } 
  
      //****************************** LIFTING *************************************************/
      if(oi.gamePad.getButtonAPressed()) { // ****************** LIFT TO LOW**************/
        if(oi.gamePad.getLeftTrigger()) {
          new MoveLiftCommand(LiftPositions.LOW, 2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else {
          new MoveLiftCommand(LiftPositions.LOW, 2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        }
      } else if(oi.gamePad.getButtonXPressed()) { 
        if(oi.gamePad.getLeftTrigger()) { //*********************LIFT TO LOW ROCKET****** */
          new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_ONE, 2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else { //***********************************************LIFT TO MID HATCH****** */
          new MoveLiftCommand(LiftPositions.HATCH_MID, 2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        }
      } else if(oi.gamePad.getButtonYPressed()) { 
        if(oi.gamePad.getLeftTrigger()) { //**********************LIFT TO CARGO MID****** */
          new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_TWO, 2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else { //***********************************************LIFT TO HIGH HATCH***** */
          new MoveLiftCommand(LiftPositions.HATCH_HIGH, 2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        }
      } else if(oi.gamePad.getButtonBPressed() && oi.gamePad.getLeftTrigger()) { // **** LIFT TO HIGH ROCKET
        new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_THREE, 2).start();
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
      } else if(oi.gamePad.getRightButtonPressed()) { // ******************* LIFT TO CARGO SHIP
        new MoveLiftCommand(LiftPositions.CARGO_SHIP, 2).start();
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
      } else if(oi.gamePad.getRightTrigger() && oi.gamePad.getLeftTrigger()) { // ****** LIFT LOADING STATION
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        new MoveLiftCommand(LiftPositions.CARGO_LOADING_STATION, 2).start();
      } else if(!lift.isMoving()) {
        lift.stallLift(RobotState.liftPosition);
      } 
  

  
      // *************************** DEPLOY ********************************************/
      if(oi.gamePad.getBackButtonPressed()) { //*********GUNNER DEPLOY********** */
        new DeployCommand(RobotState.liftPosition, 1, 2).start();
      } else if(oi.gamePad.getBackButtonReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      } else if(oi.driver.driverDeployPressed()) { //*******DRIVER DEPLOY******* */
        new DeployCommand(RobotState.liftPosition, 1, 2).start();
      } else if(oi.driver.driverDeployReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      }
  
      // ****************************** JACK ***********************************************/
      if(oi.gamePad.getPOVLeft()) {
        // start climb sequence level 2
        isClimb = true;
        isLevel3 = false;
        new PrepareClimb2().start();
      } else if(oi.gamePad.getPOVRight()) {
        // start climb sequence level 3
        isClimb = true;
        isLevel3 = true;
        new PrepareClimb3().start();
      }
    } else if(isClimb){ // ****************************************** CLIMBING ************/
      if(oi.gamePad.getPOVDown()) {
        // push down jack
        if(!isJackRunning){
          isJackRunning = true;
          if(isLevel3){
            new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstants.DOWN_STATE_LEVEL_3,true,4).start();
          }else{
            new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstants.DOWN_STATE_LEVEL_2,true,3).start();
          } 
        }
      } else if(oi.gamePad.getPOVUp()) {
        new MoveJackCommand(0,3);
        isClimb = false;
      }
    } else if(isDefenseMode) { //****************************************DEFENSE****************** */
      if(oi.gamePad.getLeftButtonPressed()) {
        new SetIntakeRollers(true, 0, 0, 1).start();
      } else if(oi.gamePad.getLeftButtonReleased()) {
        new SetIntakeRollers(true, 0, 0, 0).start();
      }

      if(oi.gamePad.getBackButtonPressed()) {
        new SetIntakeRollers(false, 0, 0, 1).start();
      } else if(oi.gamePad.getBackButtonReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
      }
    }

    //****************************CHANGE TO DEFENSE****************************************** */
    if((oi.gamePad.getLeftJoystickButton() && oi.gamePad.getRightJoystickButtonPressed())) {
      isDefenseMode = !isDefenseMode;
      if(isDefenseMode) {
        new DefenseModeCommand().start();
      } else if(!isDefenseMode) {
        new ChangeIntakeState(CargoIntakePositionState.MID).start();
      }
    }

    

    // ****************** MANUAL MODE *************************************************
    if(oi.gamePad.getStartButton()) { 
      RobotState.liftPosition = LiftPositions.MANUAL;
      double pow = oi.gamePad.getLeftJoystickY()/2;
      if(lift.getBottomHal() && pow < 0){
        pow = 0;
      }
      lift.setManualLift(pow);

      if(lift.isZero()) {
        lift.resetEncoder();
      }
    }

    
  
    //******************************* DRIVE ****************************************/
    if(isClimb){
      drive.driveFwdRotate(oi.driver.getDriverVertical()/3, 0);
      jack.setJackDriveMotor(oi.driver.getDriverVertical());
    }else{
      if(oi.visionFixButton.get()) {
        visionFixCommand.start();
      } else {
        visionFixCommand.cancel();
        Robot.gyro.gyroPIDController.setSetpoint(Pathfinder.boundHalfDegrees(Robot.gyro.getGyroAngle() + oi.driver.getDriverHorizontal() * 30.0));
        SmartDashboard.putNumber("GYRO SETPOINT", Robot.gyro.gyroPIDController.getSetpoint());
        SmartDashboard.putNumber("Gyro Value", Robot.gyro.getGyroAngle());
        Robot.gyro.gyroPIDController.enable();
        drive.driveFwdRotate(oi.driver.getDriverVertical(), Robot.gyro.getGyroPIDOutput());
      }
    }
    
  }


  public void smartdashboardTesting() {
    //***************************************************** DRIVE */
    SmartDashboard.putNumber("Left Encoder Raw", drive.getRawLeftEncoder());
    SmartDashboard.putNumber("Left Encoder Distance", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Raw", drive.getRawRightEncoder());
    SmartDashboard.putNumber("Right Encoder Distance", drive.getRightEncoderDistance());

    //***************************************************** LIFT */
    SmartDashboard.putNumber("Lift Encoder Raw", lift.getRawEncoder());
    SmartDashboard.putNumber("Lift Distance", lift.getDistance());
    SmartDashboard.putNumber("Lift Velocity", lift.getVel());
    SmartDashboard.putBoolean("Bottom Hal", lift.getBottomHal());
    SmartDashboard.putNumber("Lift Motor Current", lift.getCurrentMainMotor());
    SmartDashboard.putNumber("Lift Motor Voltage", lift.getVoltageMainMotor());

    //***************************************************** INTAKE */
    SmartDashboard.putBoolean("Up/Down Sol State", cargoIntake.getIntakeSolState());
    SmartDashboard.putBoolean("Mid State Sol State", cargoIntake.getMidStateSolState());
    SmartDashboard.putBoolean("In Hal", cargoIntake.getInHal());
    SmartDashboard.putBoolean("Out Hal", cargoIntake.getOutHal());
    cargoIntake.smartdashboard();

    //***************************************************** JACK */
    SmartDashboard.putBoolean("Jack Deployed Hal", jack.isJackAtTop());
    SmartDashboard.putBoolean("Jack Stored Hal", jack.isJackAtBottom());
    SmartDashboard.putNumber("Jack Encoder", jack.getJackEncoder());

    //***************************************************** HATCH */


    //***************************************************** VISION */


    //***************************************************** CURRENT ROBOT STATES */
    SmartDashboard.putString("Lift State", RobotState.liftPosition.toString());
    SmartDashboard.putString("Hatch State", RobotState.hatchPositionState.toString());
    SmartDashboard.putString("Intake State", RobotState.cargoIntakeState.toString());
    SmartDashboard.putString("Intaking State", RobotState.intakeMotorState.toString());
    SmartDashboard.putString("Hatch Piece State", RobotState.hatchGamePiece.toString());
    SmartDashboard.putString("Cargo Piece State", RobotState.cargoGamePiece.toString());
    SmartDashboard.putString("Hatch Deploy State", RobotState.hatchDeployState.toString());
    SmartDashboard.putString("Cargo Deploy Motor State", RobotState.cargoDeployState.toString());
  }
}
