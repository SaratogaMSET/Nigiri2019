/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.team254.lib.trajectory.Spline;
import com.team254.lib.trajectory.WaypointSequence;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.*;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.commands.intake.CargoIntakeMidToIn;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.commands.intake.WaitUntilLiftDownIntake;
import frc.robot.commands.semiauto.AutoIntakeHatch;
import frc.robot.commands.semiauto.CargoShipLiftAndIntake;
import frc.robot.commands.semiauto.DefenseModeCommand;
import frc.robot.commands.semiauto.climb.ClimbThreeJack;
import frc.robot.commands.semiauto.climb.ClimbTwoJack;
import frc.robot.commands.semiauto.climb.DeployBuddyClimbFork;
import frc.robot.commands.semiauto.climb.DeployClimbForks;
import frc.robot.commands.semiauto.climb.JackMotionProfileAndLiftCommand;
import frc.robot.commands.semiauto.climb.PrepareClimb2;
import frc.robot.commands.semiauto.climb.PrepareClimb3;
import frc.robot.commands.semiauto.climb.TestJackDriveMotors;
import frc.robot.commands.test.CargoDeployTest;
import frc.robot.commands.test.DiagnosticsCommand;
import frc.robot.commands.test.DrivetrainDiagnostic;
import frc.robot.commands.test.HatchTest;
import frc.robot.commands.test.IntakeMotorsTest;
import frc.robot.commands.test.LedTest;
import frc.robot.commands.test.LiftTest;
import frc.robot.commands.test.TestDTMaxVA;
import frc.robot.commands.test.TestTalonVelocity;
import frc.robot.commands.test.TuneMotionProfile;

import frc.robot.commands.vision.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.AutoSelector.AutoSelectorValue;
import frc.robot.subsystems.AutoSelector.Control;
import frc.robot.subsystems.AutoSelector.Side;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.subsystems.LiftSubsystem.LiftEncoderConstants;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.subsystems.LiftSubsystem.PIDConstants;
import frc.robot.util.FishyMath;
import frc.robot.util.Logging;
import frc.robot.util.pid.TalonPIDSetter;
import frc.robot.RobotState;
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
  public static boolean autoControl = true;
  public static boolean isManualMode = false;
  public static boolean isLogging = false;

  // public static ArrayList


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
  private static Subsystem[] subsystems;

  // Vision
  public static VisionSubsystem vision;
  public static VisionFixCommand visionFixCommand;
  public static VisionSplineCommand visionSplineCommand;
  public static GyroRotationalHoldCommand gholdTest;

  public static Preferences prefs;

  public static Compressor compressor;

  public static int timeoutMs = 20;


  // DRIVE kinematics tracking
  public double prev_vel = 0.0; // TODO put in diagnostic
  public double prev_time = 0.0;
  public double max_vel = 0.0;
  public double max_accel = 0.0;
  public Timer accelTime = new Timer();
  public static boolean isManualAuto;

  double lastTime;
  double loopTime;
  double loopCount;
  public static Timer intakeTime, time, loopTimer;

  public static boolean isClimb;
  public static boolean isLevel3;
  public static boolean isJackRunning;
  public static boolean doneClimb;
  public static boolean isClimbPrepared = false; // NOT CONSTANT, DONT TOUCH
  public static boolean isDoubleClimb = false; // NOT CONSTANT, DONT TOUCH

  public static SendableChooser<Command> autoChooser;
  public static Command backRocketLeft;
  public static Command backRocketRight;
  public static Command backRocketLeftSlow;
  public static Command backRocketRightSlow;
  public static Command nearRocketLeft;
  public static Command nearRocketRight;
  public static Command cargoSideRight;
  public static Command cargoSideLeft;
  public static Command closeCargoShip;
  public static Command cargoShipAuto;
  public static Command cargoSideFarLeft;
  public static Command secondLeg;

  public static Command doubleRocket;

  public static Command autoCommand;

  public static AutoSelectorValue currentAuto;
  public static AutoSelectorValue changedAuto;
  public static Side autoSide;
  public static int autoNumber;
  public Timer autoTime;

  // TEST
  public static Command testDTMaxVA = new TestDTMaxVA(20.0);
  public static Command testTalonVel = new TestTalonVelocity(20.0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // Shuffleboard.getTab("Drive").add("Time Left", Timer.getFPGATimestamp()).withSize(2, 4).withPosition(2,4)
    //                     .withWidget(BuiltInWidgets.kNumberBar).getEntry();
    //******************************************** Subsystems */
    oi = new OI();
    drive = new DrivetrainSubsystem();
    cargoDeploy = new CargoDeploySubsystem();
    cargoIntake = new CargoIntakeSubsystem();
    led = new LedSubsystem();
    jack = new JackSubsystem();
    autoSelector = new AutoSelector();
    camera = new CameraSubsystem();
    gyro = new GyroSubsystem();
    lift = new LiftSubsystem();
    hatch = new HatchSubsystem();
    compressor = new Compressor(RobotMap.PCM);
    prefs = Preferences.getInstance();
    drive.changeBrakeCoast(false);

    subsystems = new Subsystem[] {drive, cargoDeploy, cargoIntake,
      led, jack, autoSelector, camera, gyro, lift, hatch};

    try {
      vision = new VisionSubsystem();
      SmartDashboard.putString("VISION INIT", "1");
    } catch(Exception e){
      SmartDashboard.putString("VISION INIT", "0");
      e.printStackTrace();
    }
    robotState = new RobotState();

    drive.changeBrakeCoast(false);
    lift.resetEncoder();
    // jack.resetJackEncoder();

    time = new Timer();
    //LOOP COUNT
    // loopTimer = new Timer();
    // loopTimer.start();
    time.reset();
    time.start();
    // lastTime = time.get();
    // loopCount = 1;
    doneClimb = false;

    // backRocketLeft = new IanAssistedDrive(false, false);
    // backRocketRight = new IanAssistedDrive(true, false);

    // backRocketLeftSlow = new IanAssistedDrive(false, true);
    // backRocketRightSlow = new IanAssistedDrive(true, true);

    // nearRocketLeft = new NearRocket(false);
    // nearRocketRight = new NearRocket(true);

    // closeCargoShip = new ForwardAuto(true);
    // cargoShipAuto = new ForwardAuto(false);

    // cargoSideLeft = new CargoShipSide(false);
    // cargoSideRight = new CargoShipSide(true);

    // cargoSideFarLeft = new CargoShipSideMid(false);


    Robot.gyro.resetGyro();

    // Robot.gyro.gyro.zeroYaw();
    // (new Thread(RobotPose.getRunnable())).start();
    // visionSplineCommand = new VisionSplineCommand();
    visionFixCommand = new VisionFixCommand();
    gholdTest = new GyroRotationalHoldCommand();

    doubleRocket = new DoubleRocket(true);

    autoTime = new Timer();
    currentAuto = new AutoSelectorValue(autoSelector.getSide(), autoSelector.getAutoPotNumber());
    changedAuto = currentAuto;

    SelectAuto.chooseAuto(autoSelector.getAutoPotNumber(), autoSelector.getSide());
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
    // if(loopTimer.get() > 5) {
      // loopTime = 0;
      // loopCount = 1;
    // }
    // loopTime += (time.get() - lastTime);
    // SmartDashboard.putNumber("LoopTime", loopTime/loopCount);
    // lastTime = time.get();
    // loopCount += 1;
    //***********************************UPDATE STATES***************************** */
    RobotState.cargoIntakeState = cargoIntake.updateIntakeState();
    RobotState.intakeMotorState = cargoIntake.updateIntakeRollerState();
    RobotState.cargoGamePiece = cargoDeploy.updateCargoGamePieceState();
    RobotState.cargoDeployState = cargoDeploy.updateCargoDeployState();
    RobotState.hatchPositionState = hatch.updateHatchPositionState();
    RobotState.hatchDeployState = hatch.updateHatchDeployState();
    RobotState.liftPosition = lift.updateLiftPosition();
    smartdashboardTesting();

    // DEBUG
    // SmartDashboard.putNumber("GYRO", Robot.gyro.getGyroAngle());
    // SmartDashboard.putNumber("LEFT ENCODER", Robot.drive.getLeftEncoderDistance());

    // lift.smartdashCurrent();
    // jack.smartdas                                                hCurrent();

    //*********************************CHECK STATE VALIDITY*********************** */
    // cargoIntake.checkIntakeState();
    // hatch.checkHatchStateValid();
    // SmartDashboard.putNumber("JACK ENCODER", jack.getJackEncoder());
    // SmartDashboard.putBoolean("Is Defense Mode", isDefenseMode);
    // SmartDashboard.putBoolean("Is Hatch Aquired", hatch.getHatchAcquired());

    // SmartDashboard.putString("Auto", autoSelector.getAuto());

    String side = autoSelector.getSide() == AutoSelector.Side.LEFT ? "Left" : "Right";
    String control = autoSelector.getControl() == AutoSelector.Control.TELEOP ? "Teleop" : "Auto";
    SmartDashboard.putString("Side", side);
    SmartDashboard.putString("Control", control);
    SmartDashboard.putNumber("Auto Rotary", autoSelector.getPotVoltage());
    SmartDashboard.putNumber("Auto Rotary Number", autoSelector.getAutoPotNumber());

    // Important Front Page DS Stuff
    SmartDashboard.putString("Auto", autoControl ? side + " Rocket Auto" : "Teleop");
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

    // Safety Checks
    if(jack.getJackEncoder() > 100){
      if(!isClimb) {
        jack.setJackMotor(-0.08);
        SmartDashboard.putNumber("Jack Stall Power", 0.08);
      } 
      // jack.setJackMotorMP(JackSubsystem.JackEncoderConstants.UP_STATE);
      if(jack.isJackAtTop()) {
        jack.resetJackEncoder();
      }
    } else if(jack.getJackEncoder() < 50) {
      if(!isClimb) {
        jack.setJackMotor(0);
        SmartDashboard.putNumber("Jack Stall Power", 0.0);
      }
    }


    // SmartDashboard.putNumber("Right Encoder", Robot.drive.getRightEncoderDistance());
    // SmartDashboard.putNumber("Raw Right Encoder", Robot.drive.getRawRightEncoder());
    // SmartDashboard.putNumber("Left Encoder", Robot.drive.getLeftEncoderDistance());
    // SmartDashboard.putNumber("Raw Left Encoder", Robot.drive.getRawLeftEncoder());

    SmartDashboard.putNumber("Gyro Angle", Robot.gyro.getGyroAngle());

    SmartDashboard.putNumber("ZZ ROBOT X", RobotPose.getX());
    SmartDashboard.putNumber("ZZ ROBOT Y", RobotPose.getY());
    SmartDashboard.putNumber("ZZ ROBOT HEADING", FishyMath.r2d(RobotPose.getHeading()));
    SmartDashboard.putNumber("ZZ ROBOT POSE DT", RobotPose.getRunnable().getDt());


    // Vision
    if(vision != null) {
      SmartDashboard.putBoolean("Is Vision", true);
      if(visionFixCommand == null || (visionFixCommand != null && !visionFixCommand.isRunning())) {
        vision.readData();
      }

      Double angle = vision.getAngle();
      Double dist = vision.getDistance();
      if(angle != null) {
        if(Math.abs(angle) < 3.0) {
          LedSubsystem.led.set(0.61);
          // SmartDashboard.putBoolean("VSTATUS", true);
        } else {
          LedSubsystem.led.set(0.87);
          // SmartDashboard.putBoolean("VSTATUS", false);
        }
      } else {
        // led.solidRed(1);
        // led.chase(0);
      }
      if(dist != null) {
        SmartDashboard.putNumber("VISION DISTANCE", dist);
      }
      SmartDashboard.putNumber("VISION DISTANCE", -1.0);
    } else {
      LedSubsystem.led.set(.11);
      SmartDashboard.putBoolean("Is Vision", false);
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
    autoControl = true;
    autoTime.stop();

    // Robot.autoCommand = new TuneMotionProfile("StraightFastShort");


    isManualAuto = false;
    init(autoControl);
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    compressor.stop();
    if(autoControl) {
      if(oi.driver.getDriverButton8()) {
        // Auto Kill Switch and Start Teleop
        led.chase(0);
        Scheduler.getInstance().removeAll();
        stopAll();
        autoControl = false;
        init(autoControl);
        teleopLoop();
      } else if (oi.driver.getDriverButton9()) {
        stopAll();
        autoControl = false;
        init(autoControl);
      } else {

      }

    } else if(oi.driver.getDriverButton3Pressed()) {
      if(secondLeg != null) {
        autoControl = true;
        secondLeg.start();
      }
    } else {
      teleopLoop();
    }
  }

  public void stopAll() {
    isManualAuto = true;
    for(Subsystem s : subsystems) {s.stopAll();}
  }

  @Override
  public void teleopInit() {
    autoTime.stop();
    init(false);
  }

  //Init method for both auto and teleop
  public static void init(boolean auto) {
    if(auto) {
      gyro.resetGyro();
      drive.resetEncoders();
      doubleRocket.start();
      // new SelectAuto().start();
      // new DoubleCargoShip(false).start();
      // new TestTalonVelocity(10.0).start();
      // new TestDTMaxVA(20.0).start();
      // new MotionProfileCommand("StraightSlowLong", 0.0).start();
      // new GyroPIDCommand(90.0, 10.0).start();
      // new TuneMotionProfile("StrightSlowLong").start();
      // new MotionProfileCommand("StrightSlowLong").start();
    } else {
      Robot.drive.rawDrive(0.0, 0.0);
      drive.changeBrakeCoast(false);

      // visionFixCommand = new VisionFixCommand();

      compressor.setClosedLoopControl(true);
      compressor.start();

      time.reset();
      time.start();
      if (isLogging) {
        Logging.createLogFile();
      }
      isClimb = false;
      isLevel3 = false;
      isJackRunning = false;
      jack.setJackMPVals(true);
      autoControl = false; 

      // new ChangeIntakeState(CargoIntakePositionState.MID).start();
      // new DiagnosticsCommand().start();
      // new LiftTest().start();
      // new DrivetrainDiagnostic().start();
      // new IntakeMotorsTest().start();
      // new HatchTest().start();
      // new LedTest().start();
      // new CargoDeployTest().start();
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

    testDTMaxVA.start();

  }

  /**
   *
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();

  }

  public static void switchAutoToTeleop() {
    if(autoControl) {
      Scheduler.getInstance().removeAll();
      // The error should be fixed, if ur using the robot test the stopAll() method
      // stopAll();
      autoControl = false;
      init(autoControl);
    }
  }

  @Override
  public void disabledInit() {
    // Remove all commands from queue
    Scheduler.getInstance().removeAll();
    stopAll();
    if(isLogging) {
      Logging.closeWriter();
    }

    Robot.gyro.driverGyroPID.disable();

  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().removeAll();
    stopAll();
    if(isLogging) {
      Logging.closeWriter();
    }

    if(autoSelector.getControl() == Control.AUTO) {
      // SmartDashboard.putBoolean("In the Selector", true);
      autoSide = autoSelector.getSide();
      autoNumber = autoSelector.getAutoPotNumber();
      if(!currentAuto.equals(autoSide, autoNumber)) {
        if(!changedAuto.equals(autoSide, autoNumber)) {
          changedAuto = new AutoSelectorValue(autoSide, autoNumber);
          autoTime.reset();
          autoTime.start();
        } else {
          if(autoTime.get() > 0.5) {
            SelectAuto.chooseAuto(autoNumber, autoSide);
            currentAuto = new AutoSelectorValue(autoSide, autoNumber);
            autoTime.stop();
            autoTime.reset();
          }
        }
      } else {
        autoTime.stop();
        autoTime.reset();
      }
    }
    SmartDashboard.putNumber("AutoTimer", autoTime.get());
  }

  public void teleopLoop() {
    //******************************* INTAKE ***********************************************/
    if(!isClimb && !isDefenseMode){
      if(oi.gamePad.getLeftButtonPressed()) { //****************RUN INTAKES*********** */
        if(RobotState.liftPosition != LiftPositions.LOW
          && !lift.isMoving())  {

          if(oi.gamePad.getStartButton()) {
            cargoIntake.switchSol(true);
          } else {
            new SetIntakeRollers(true, 0, 0, 1).start(); // only carriage
          }
        } else {
          if(oi.gamePad.getStartButton()) {
            cargoIntake.switchSol(true);
          } else {
            new ChangeIntakeState(CargoIntakePositionState.OUT).start();
            new WaitUntilLiftDownIntake(true, 1, 1, 1, 3).start();
          }
        }
        compressor.stop();
      } else if(oi.gamePad.getLeftButtonReleased()) { // *******STOP AND STORE********* */
        if(oi.gamePad.getStartButton()) {
          cargoIntake.switchSol(false);
        } else {
          new ChangeIntakeState(CargoIntakePositionState.MID).start();
          new SetIntakeRollers(false, 0, 0, 0).start();
        }
        compressor.start();
      }

      //****************************** LIFTING *************************************************/
      if(!doneClimb) { // Cannot move lift when we are done with the climb
        if (oi.gamePad.getButtonAPressed()) { // ****************** LIFT TO LOW**************/
          if (oi.gamePad.getLeftTrigger()) {
            new MoveLiftCommand(LiftPositions.LOW, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
          } else {
            new MoveLiftCommand(LiftPositions.LOW, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
          }
        } else if (oi.gamePad.getButtonXPressed()) {
          if (oi.gamePad.getLeftTrigger()) { // *********************LIFT TO LOW ROCKET****** */
            new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_ONE, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
          } else { // ***********************************************LIFT TO MID HATCH****** */
            // new MoveLiftCommand(LiftPositions.HATCH_MID, 1.2).start();
            new MoveLiftCommand(LiftPositions.LOW_HATCH, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
          }
        } else if (oi.gamePad.getButtonYPressed()) {
          if (oi.gamePad.getLeftTrigger()) { // **********************LIFT TO CARGO MID****** */
            new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_TWO, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
          } else { // ***********************************************LIFT TO HIGH HATCH***** */
            // new MoveLiftCommand(LiftPositions.HATCH_HIGH, 1.2).start();
            new MoveLiftCommand(LiftPositions.HATCH_MID, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
          }
        } else if (oi.gamePad.getButtonBPressed()) { // **** LIFT TO HIGH ROCKET
          if(oi.gamePad.getLeftTrigger()) {
            new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_THREE, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
          } else {
            new MoveLiftCommand(LiftPositions.HATCH_HIGH, 1.2).start();
            new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
          }
        } else if (oi.gamePad.getRightButtonPressed()) { // ******************* LIFT TO CARGO SHIP
          new CargoShipLiftAndIntake().start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else if (oi.gamePad.getRightTrigger() && oi.gamePad.getLeftTrigger()) { // ****** LIFT LOADING STATION
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
          new MoveLiftCommand(LiftPositions.CARGO_LOADING_STATION, 1.2).start();
        } else if (!RobotState.isRunningLiftCommand && !RobotState.isManualLift) {
          lift.stallLift(RobotState.liftPosition);
        }
      }

      // *************************** DEPLOY ********************************************/
      if(oi.gamePad.getBackButtonPressed()) { //*********GUNNER DEPLOY********** */
        new SetIntakeRollers(false, 0, 0, 1).start();
        new DeployHatchCommand().start();
      } else if(oi.gamePad.getBackButtonReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      } else if(oi.driver.driverDeployPressed()) {
        new DeployCommand(RobotState.lastLiftTarget, 1).start();
      } else if(oi.driver.driverDeployReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      }

      //********TAKE OUT LATER*********TAKE OUT LATER****** TAKE OUT LATER************* */
      // if(oi.gamePad.getLeftJoystickButtonPressed()) {
      //   new CargoIntakeMidToIn().start();
      // }

      // ****************************** JACK ***********************************************/
      if(oi.gamePad.getPOVLeft()) {
        // start climb sequence level 2
        isClimb = true;
        isLevel3 = false;
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        new PrepareClimb2().start();
      } else if(oi.gamePad.getPOVRight()) {
        // start climb sequence level 3
        isClimb = true;
        isLevel3 = true;
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        new PrepareClimb3().start();
      }
    } else if(isClimb){ // ****************************************** CLIMBING ************/
      compressor.stop();
      // *************************** DEPLOY ********************************************/
      if(oi.gamePad.getBackButtonPressed()) { //*********GUNNER DEPLOY********** */
        new SetIntakeRollers(false, 0, 0, 1).start();
        new DeployHatchCommand().start();
      } else if(oi.gamePad.getBackButtonReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      } else if(oi.driver.driverDeployPressed()) {
        new DeployCommand(RobotState.lastLiftTarget, 1).start();
      } else if(oi.driver.driverDeployReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      }
      if(oi.gamePad.getLeftButtonPressed()) { //****************INTAKES OUT*********** */
        if(oi.gamePad.getStartButton()) {
          cargoIntake.switchSol(true);
        }
      } else if(oi.gamePad.getLeftButtonReleased()) { // *******INTAKES STORE********* */
        if(oi.gamePad.getStartButton()) {
          cargoIntake.switchSol(false);
        } 
      }

      if(isClimbPrepared) {
        if(oi.gamePad.getPOVLeft()) {
          new MoveLiftCommand(LiftPositions.CLIMB_HAB_TWO_TOL, 1.2).start();
          isLevel3 = false;
        } else if(oi.gamePad.getPOVRight()) {
          new MoveLiftCommand(LiftPositions.CLIMB_HAB_THREE, 1.2).start();
          isLevel3 = true;
        }
        if(Robot.isJackRunning) {
          if(oi.gamePad.getRightTrigger() && !Robot.isDoubleClimb) {
            Robot.isDoubleClimb = true;
            new DeployBuddyClimbFork().start();
          }
        }
        if(oi.gamePad.getPOVDown()) {
          // push down jack
          if(!isJackRunning){
            isJackRunning = true;
            if(isLevel3){
              new ClimbThreeJack().start();
            }else{
              new ClimbTwoJack().start();
            }
          }
  
        } else if(oi.gamePad.getPOVUp()) {
          doneClimb = true;
          // new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
          // new ChangeIntakeState(CargoIntakePositionState.OUT).start();
          jack.setJackMPVals(false);
          if(Robot.isDoubleClimb) {
            new MoveJackCommand(Robot.jack.getJackEncoder() - 2200, 3).start();
          }
          else {
            new MoveJackCommand(JackSubsystem.JackEncoderConstants.UP_STATE,3).start();
          }
          // isClimb = false;
        }


      }
    } else if(isDefenseMode) { //*********************************DEFENSE****************** */
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
    if((oi.gamePad.getRightJoystickButtonPressed())) {
      isDefenseMode = !isDefenseMode;
      if(isDefenseMode) {
        new DefenseModeCommand().start();
      } else if(!isDefenseMode) {
        new ChangeIntakeState(CargoIntakePositionState.MID).start();
      }
    }



    // ****************** MANUAL MODE *************************************************
    if(oi.gamePad.getStartButton()) {
      RobotState.isManualLift = true;
      int sign = oi.gamePad.getLeftJoystickY() > 0 ? 1 : -1;
      double pow = Math.pow(oi.gamePad.getLeftJoystickY()/2, 2) * sign;
      // SmartDashboard.putNumber("Manual Lift Power", pow);
      if(lift.getBottomHal() && pow < 0){
        pow = 0;
      }
      lift.setManualLift(pow);

      if(lift.getBottomHal()) {
        lift.resetEncoder();
      }
    } else if(RobotState.isManualLift) {
      lift.setManualLift(0);
    }



    //******************************* DRIVE ****************************************/
    if(isJackRunning){
      drive.driveFwdRotate(oi.driver.getDriverVertical()/2, 0);
      jack.setJackDriveMotor(oi.driver.getDriverVertical());
    }else{
      if(vision != null && visionFixCommand != null && oi.visionFixButton.get()) {
        if(!visionFixCommand.isRunning()){
          visionFixCommand.start();
        }
        // if(!gholdTest.isRunning()) {
        //   gholdTest.setTargetAngle(gyro.getGyroAngle() + 45.0);
        //   gholdTest.start();
        // }
      }
      else {
        if(visionFixCommand != null) visionFixCommand.cancel();
        if(gholdTest != null) gholdTest.cancel();
        
        // TUNED VALUE
        Robot.gyro.driverGyroPID.setSetpoint(FishyMath.d2r(oi.driver.getDriverHorizontal() * 120.0));
        Robot.gyro.driverGyroPID.enable();

        Robot.drive.joystickPID.setSetpoint(oi.driver.getDriverVertical());
        Robot.drive.joystickPID.enable();

        SmartDashboard.putNumber("GYRO SPEED", FishyMath.r2d(gyro.gyro.getRate()));
        SmartDashboard.putNumber("JOY VERT", Robot.drive.pidInputPower);

        drive.driveFwdRotate(Robot.drive.pidInputPower, Robot.gyro.driverPIDOutput);
        // drive.rawDrive(Robot.gyro.driverPIDOutput, -Robot.gyro.driverPIDOutput);
      }
    }

  }


  public void smartdashboardTesting() {
    //***************************************************** DRIVE */
    SmartDashboard.putNumber("Left Encoder Raw", drive.getRawLeftEncoder());
    // SmartDashboard.putNumber("Left Encoder Distance", drive.getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder Raw", drive.getRawRightEncoder());
    // SmartDashboard.putNumber("Right Encoder Distance", drive.getRightEncoderDistance());
    // SmartDashboard.putNumber("Right Velocity", drive.getRightEncoderVelocity());
    // SmartDashboard.putNumber("Left Velocity", drive.getLeftEncoderVelocity());

    //***************************************************** LIFT */
    SmartDashboard.putNumber("Lift Encoder Raw", lift.getRawEncoder());
    SmartDashboard.putNumber("Lift Distance", lift.getDistance());
    // SmartDashboard.putNumber("Lift Velocity", lift.getVel());
    // SmartDashboard.putBoolean("Bottom Hal", lift.getBottomHal());
    // SmartDashboard.putBoolean("isMoving", lift.isMoving());
    // SmartDashboard.putNumber("Lift Motor Current", lift.getCurrentMainMotor());
    // SmartDashboard.putNumber("Lift Motor Voltage", lift.getVoltageMainMotor());

    //***************************************************** INTAKE */
    // SmartDashboard.putBoolean("Up/Down Sol State", cargoIntake.getIntakeSolState());
    // SmartDashboard.putBoolean("Mid State Sol State", cargoIntake.getMidStateSolState());
    // SmartDashboard.putBoolean("In Hal", cargoIntake.getInHal());
    // SmartDashboard.putBoolean("Out Hal", cargoIntake.getOutHal());
    // cargoIntake.smartdashboard();

    //***************************************************** JACK */
    // SmartDashboard.putBoolean("Jack Deployed Hal", jack.isJackAtTop());
    // SmartDashboard.putBoolean("Jack Stored Hal", jack.isJackAtBottom());
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
