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
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.*;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.commands.intake.CargoIntakeMidToIn;
import frc.robot.commands.intake.ChangeIntakeState;
import frc.robot.commands.intake.SetIntakeRollers;
import frc.robot.commands.intake.WaitUntilLiftDownIntake;
import frc.robot.commands.semiauto.CargoShipLiftAndIntake;
import frc.robot.commands.semiauto.DefenseModeCommand;
import frc.robot.commands.semiauto.climb.ClimbThreeJack;
import frc.robot.commands.semiauto.climb.ClimbTwoJack;
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
import frc.robot.commands.vision.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CargoIntakeSubsystem.CargoIntakePositionState;
import frc.robot.subsystems.HatchSubsystem.HatchPositionState;
import frc.robot.subsystems.LiftSubsystem.LiftEncoderConstants;
import frc.robot.subsystems.LiftSubsystem.LiftPositions;
import frc.robot.subsystems.LiftSubsystem.PIDConstants;
import frc.robot.util.FishyMath;
import frc.robot.util.Logging;
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

  public static Preferences prefs;

  public static Compressor compressor;

  public static int timeoutMs = 20;


  // DRIVE kinematics tracking
  public double prev_vel = 0.0; // TODO put in diagnostic
  public double prev_time = 0.0;
  public double max_vel = 0.0;
  public double max_accel = 0.0;
  public Timer accelTime = new Timer();

  double lastTime;
  double loopTime;
  double loopCount;
  public static Timer intakeTime, time, loopTimer;

  public static boolean isClimb;
  public static boolean isLevel3;
  public static boolean isJackRunning;
  public static boolean doneClimb;
  public static boolean isClimbPrepared = false;

  public static Command autoCommandLeft;
  public static Command autoCommandRight;

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
    compressor = new Compressor(4);
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
    jack.resetJackEncoder();

    time = new Timer();
    //LOOP COUNT
    // loopTimer = new Timer();
    // loopTimer.start();
    time.reset();
    time.start();
    // lastTime = time.get();
    // loopCount = 1;
    doneClimb = false;

    autoCommandLeft = new IanAssistedDrive(false);
    autoCommandRight = new IanAssistedDrive(true);

    Robot.gyro.resetGyro();
    Robot.gyro.gyro.setAngleAdjustment(180.0);
    // Robot.gyro.gyro.zeroYaw();
    (new Thread(RobotPose.getRunnable())).start();
    visionSplineCommand = new VisionSplineCommand();
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
    
    // Important Front Page DS Stuff
    SmartDashboard.putString("Auto", autoControl ? side + " Rocket Auto" : "Teleop");
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

    // Safety Checks
    if(!jack.isJackAtTop() && !isClimb){
      // jack.setJackMotorMP(JackSubsystem.JackEncoderConstants.UP_STATE);
      if(jack.isJackAtTop()) {
        jack.resetJackEncoder();
      }
    }

    SmartDashboard.putNumber("ZZ ROBOT X", RobotPose.getX());
    SmartDashboard.putNumber("ZZ ROBOT Y", RobotPose.getY());
    SmartDashboard.putNumber("ZZ ROBOT HEADING", FishyMath.r2d(RobotPose.getHeading()));
    SmartDashboard.putNumber("ZZ ROBOT POSE DT", RobotPose.getRunnable().getDt());

    // Vision
    // if(vision != null) {
    //   SmartDashboard.putBoolean("Is Vision", true);
    //   if(visionFixCommand == null || (visionFixCommand != null && !visionFixCommand.isRunning())) {
    //     vision.readData();
    //   }

    //   Double angle = vision.getAngleDisplacement();
    //   Double dist = vision.getDistance();
    //   if(angle != null) {
    //     if(Math.abs(angle) < 3.0) {
    //       LedSubsystem.led.set(0.61);
    //       // SmartDashboard.putBoolean("VSTATUS", true);
    //     } else {
    //       LedSubsystem.led.set(0.87);
    //       // SmartDashboard.putBoolean("VSTATUS", false);
    //     }
    //   } else {
    //     // led.solidRed(1);
    //     // led.chase(0);
    //   }
    //   if(dist != null) {
    //     SmartDashboard.putNumber("VISION DISTANCE", dist);
    //   }
    //   SmartDashboard.putNumber("VISION DISTANCE", -1.0);
    // } else {
    //   LedSubsystem.led.set(.11);
    //   SmartDashboard.putBoolean("Is Vision", false);
    // }

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
    autoControl = autoSelector.getControl() == AutoSelector.Control.AUTO;
    init(autoControl);
    // Stop putting all your code here and put it in the init() method–don't override this shit
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

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
        // Put all reg auto periodic shit here
      }
    } else {
      teleopLoop();
    }
  }

  public void stopAll() {
    autoCommandLeft.cancel();
    autoCommandRight.cancel();
    for(Subsystem s : subsystems) {s.stopAll();}
  }

  @Override
  public void teleopInit() {
    init(false);
  }

  //Init method for both auto and teleop
  public static void init(boolean auto) {
    if(auto) {
      gyro.resetGyro();
      drive.resetEncoders();
      if(autoSelector.getSide() == AutoSelector.Side.RIGHT) {
        autoCommandRight.start();
      }
      else {
        autoCommandLeft.start();
      }
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
      // new ChangeIntakeState(CargoIntakePositionState.MID).start();
      // new DiagnosticsCommand().start();
      // new LiftTest().start();
      // new DrivetrainDiagnostic().start();
      // new IntakeMotorsTest().start();
      // new HatchTest().start();
      // new LedTest().start();
      // new CargoDeployTest().start();

    //   Spline test = new Spline();
    //   Spline.reticulateSplines(new WaypointSequence.Waypoint(0, 0, FishyMath.d2r(30)), new WaypointSequence.Waypoint(13, -2.833333333333, FishyMath.d2r(0)), test, Spline.QuinticHermite);
    //   (new PurePursuitCommand(test)).start();
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
   *
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();

    // gyro.gyroPIDController.enable();
    // double pidOut = gyro.getGyroPIDOutput();
    // drive.rawDrive(pidOut, -pidOut);

  }

  public static void switchAutoToTeleop() {
    if(autoControl) {
      Scheduler.getInstance().removeAll();
      // The error should be fixed, if ur using the robot test the stopAll() method
      //stopAll();
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
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().removeAll();
    stopAll();
    if(isLogging) {
      Logging.closeWriter();
    }
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

      // Added stopping compressor to high heights(hatch mid and above)
      //****************************** LIFTING *************************************************/
      if(oi.gamePad.getButtonAPressed()) { // ****************** LIFT TO LOW**************/
        if(oi.gamePad.getLeftTrigger()) {
          new MoveLiftCommand(LiftPositions.LOW, 1.2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else {
          new MoveLiftCommand(LiftPositions.LOW, 1.2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        }
      } else if(oi.gamePad.getButtonXPressed()) {
        if(oi.gamePad.getLeftTrigger()) { //*********************LIFT TO LOW ROCKET****** */
          new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_ONE, 1.2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else { //***********************************************LIFT TO MID HATCH****** */
          new MoveLiftCommand(LiftPositions.HATCH_MID, 1.2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        }
      } else if(oi.gamePad.getButtonYPressed()) {
        if(oi.gamePad.getLeftTrigger()) { //**********************LIFT TO CARGO MID****** */
          new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_TWO, 1.2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        } else { //***********************************************LIFT TO HIGH HATCH***** */
          new MoveLiftCommand(LiftPositions.HATCH_HIGH, 1.2).start();
          new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        }
      } else if(oi.gamePad.getButtonBPressed() && oi.gamePad.getLeftTrigger()) { // **** LIFT TO HIGH ROCKET
        new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_THREE, 1.2).start();
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
      } else if(oi.gamePad.getRightButtonPressed()) { // ******************* LIFT TO CARGO SHIP
        new CargoShipLiftAndIntake().start();
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
      } else if(oi.gamePad.getRightTrigger() && oi.gamePad.getLeftTrigger()) { // ****** LIFT LOADING STATION
        new MoveHatchCommand(HatchPositionState.HATCH_IN).start();
        new MoveLiftCommand(LiftPositions.CARGO_LOADING_STATION, 1.2).start();
      } else if(!RobotState.isRunningLiftCommand) {
        lift.stallLift(RobotState.liftPosition);
      }

      // *************************** DEPLOY ********************************************/
      if(oi.gamePad.getBackButtonPressed() || oi.driver.driverDeployPressed()) { //*********GUNNER DEPLOY********** */
        new DeployCommand(RobotState.liftPosition, 1, 2).start();
      } else if(oi.gamePad.getBackButtonReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      } if(oi.driver.driverDeployPressed()) {
        new DeployCommand(RobotState.liftPosition, 1, 2).start();
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
      if(oi.gamePad.getBackButtonPressed() || oi.driver.driverDeployPressed()) { //*********GUNNER DEPLOY********** */
        new DeployCommand(RobotState.liftPosition, 1, 2).start();
      } else if(oi.gamePad.getBackButtonReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      } if(oi.driver.driverDeployPressed()) {
        new DeployCommand(RobotState.liftPosition, 1, 2).start();
      } else if(oi.driver.driverDeployReleased()) {
        new SetIntakeRollers(false, 0, 0, 0).start();
        hatch.hatchDeployIn();
      }
      if(isClimbPrepared) {
        if(oi.gamePad.getPOVLeft()) {
          new MoveLiftCommand(LiftPositions.CLIMB_HAB_TWO_TOL, 1.2).start();
          isLevel3 = false;
        } else if(oi.gamePad.getPOVRight()) {
          new MoveLiftCommand(LiftPositions.CLIMB_HAB_THREE, 1.2).start();
          isLevel3 = true;
        }
      }
      if(oi.gamePad.getPOVDown()) {
        // push down jack
        if(!isJackRunning){
          isJackRunning = true;
          if(isLevel3){
            new ClimbThreeJack().start();
            // new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstants.DOWN_STATE_LEVEL_3, LiftEncoderConstants.CLIMB_HAB_THREE, true, 100).start();
          }else{
            // new JackMotionProfileAndLiftCommand(JackSubsystem.JackEncoderConstants.DOWN_STATE_LEVEL_2,true,3).start();
            new ClimbTwoJack().start();
          }
        }
        
      } else if(oi.gamePad.getPOVUp()) {
        doneClimb = true;
        new MoveHatchCommand(HatchPositionState.HATCH_OUT).start();
        new ChangeIntakeState(CargoIntakePositionState.OUT).start();
        jack.setJackMPVals(false);
        new MoveJackCommand(0,3).start();
        isClimb = false;
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
      double pow = oi.gamePad.getLeftJoystickY()/2;
      if(lift.getBottomHal() && pow < 0){
        pow = 0;
      }
      lift.setManualLift(pow);

      if(lift.getBottomHal()) {
        lift.resetEncoder();
      }
    }



    //******************************* DRIVE ****************************************/
    if(isJackRunning){
      drive.driveFwdRotate(oi.driver.getDriverVertical()/3, 0);
      jack.setJackDriveMotor(oi.driver.getDriverVertical());
    }else{
      if(visionSplineCommand != null && oi.visionFixButton.get()) {
        if(!visionSplineCommand.isRunning()){
          visionSplineCommand.start();
        }
      }
      else {
        if(visionSplineCommand != null) visionSplineCommand.cancel();
        Robot.gyro.driverGyroPID.setSetpoint(FishyMath.boundThetaNeg180to180(Robot.gyro.getGyroAngle() + oi.driver.getDriverHorizontal() * 20.0));
        Robot.gyro.driverGyroPID.enable();
        drive.driveFwdRotate(oi.driver.getDriverVertical(), Robot.gyro.driverPIDOutput);
      }
    }

  }


  public void smartdashboardTesting() {
    //***************************************************** DRIVE */
    // SmartDashboard.putNumber("Left Encoder Raw", drive.getRawLeftEncoder());
    // SmartDashboard.putNumber("Left Encoder Distance", drive.getLeftEncoderDistance());
    // SmartDashboard.putNumber("Right Encoder Raw", drive.getRawRightEncoder());
    // SmartDashboard.putNumber("Right Encoder Distance", drive.getRightEncoderDistance());

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
