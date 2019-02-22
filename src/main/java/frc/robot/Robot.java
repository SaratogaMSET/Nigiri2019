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
import frc.robot.commands.GyroStraightDistancePID;
import frc.robot.commands.intake.SetCargoIntakes;
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
    robotState = new RobotState();
    drive = new DrivetrainSubsystem();
    cargoDeploy = new CargoDeploySubsystem();
    cargoIntake = new CargoIntakeSubsystem();
    led = new LedSubsystem();
    jack = new JackSubsystem();
    camera = new CameraSubsystem();
    gyro = new GyroSubsystem();
    lift = new LiftSubsystem();
    // hatch = new HatchSubsystem();
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
    SmartDashboard.putNumber("TIME", accelTime.get());

    SmartDashboard.putNumber("SAMPLE RATE", drive.leftEncoder.getSamplesToAverage());
    SmartDashboard.putNumber("SAMPLE RATE R", drive.rightEncoder.getSamplesToAverage());
    SmartDashboard.putNumber("JACK ENCODER", jack.getJackEncoder());
    SmartDashboard.putNumber("LIFT ENCODER", lift.getRawEncoder());

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

    compressor.setClosedLoopControl(true);
    compressor.start();
    intakeTime = new Timer();
    // intakeTime.reset();
    // intakeTime.start();
    // cargoIntake.isOut = false;

    lift.resetEncoder();
    // initLiftTune();

    // new IntakeMotorsTest().start();
    // new LiftTest().start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // gyroStraightTest();
    SmartDashboard.putBoolean("IsOut", cargoIntake.solOut());
    // testJacks();

    // if(oi.gamePad.getButtonAPressed()) {
    //   new SetIntakePistons(true, 2).start();
    // } else if(oi.gamePad.getButtonYPressed()) {
    //   new SetIntakePistons(false, 2).start();
    // }

    // if(oi.gamePad.getButtonBPressed()) {
    //   new SetMidStatePistons(true, 2).start();
    // } else if(oi.gamePad.getButtonXPressed()) {
    //   new SetMidStatePistons(false, 2).start();
    // }

    liftTune();
    // DO NOT TOUCH OR COMMENT OUT
    // NOTE: THE VISION FIX COMMAND OVVERRIDES THE STANDARD TELEOP ARCADE DRIVING.
    // if(oi.visionFixButton.get()){
    //   visionFixCommand.start();
    //   return;
    // }
    // else {
    //   visionFixCommand.cancel();
    //   drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
    // }
    SmartDashboard.putBoolean("Intake UP Hal", cargoIntake.getInHal());
    SmartDashboard.putBoolean("Intake OUT Hal", cargoIntake.getOutHal());
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
    // SmartDashboard.putNumber("Delta X", deltaX);
    // SmartDashboard.putNumber("Delta Y", deltaY);
  }



  

    public void liftTune() {
      if(oi.gamePad.getButtonB() && oi.gamePad.getRightTrigger() > 0.5 && !lift.getIsMoving()) {
        new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_ONE, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.gamePad.getButtonY() && oi.gamePad.getRightTrigger() > 0.5 && !lift.getIsMoving()) {
        new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_TWO, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.gamePad.getButtonX() && oi.gamePad.getRightTrigger() > 0.5 && !lift.getIsMoving()) {
        new MoveLiftCommand(LiftPositions.CARGO_SHIP, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.gamePad.getButtonA() && oi.gamePad.getRightTrigger() > 0.5 && !lift.getIsMoving()) {
        new MoveLiftCommand(LiftPositions.LOW, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(!lift.getIsMoving()) {
        if(lift.getBottomHal()) {
          lift.setManualLift(0);
        } else {
          lift.setManualLift(oi.gamePad.getLeftJoystickY()/2.0);
        }
        SmartDashboard.putBoolean("Is Motion Magic", false);
      }

      if(oi.gamePad.getRightButton()) {
        new SetIntakeRollers(1, true).start();
      } else if(oi.gamePad.getLeftButton()) {
        new SetIntakeRollers(1, false).start();
      } else {
        new SetIntakeRollers(0, false).start();
      }

      if(oi.driver.getDriverButton2()) {
        lift.resetEncoder();
        lift.setPosition(LiftPositions.LOW);
      }
      drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());

      SmartDashboard.putNumber("Lift Pos", lift.getRawEncoder());
      SmartDashboard.putString("LiftPosition", lift.getLiftPosition().toString());
      SmartDashboard.putNumber("Lift Distance", lift.getDistanceFromTicks());
    }

    public void teleopLoop() {
      //**************************************************INTAKE UP/DOWN */
      if(oi.gamePad.getButtonAPressed()) {
      new SetCargoIntakes(CargoIntakeState.OUT).start();
      } else if(oi.gamePad.getButtonBPressed()) {
      new SetCargoIntakes(CargoIntakeState.MID).start();
      } else if(oi.gamePad.getButtonYPressed()) {
      new SetCargoIntakes(CargoIntakeState.IN).start();
      }
      //*************************************************INTAKE WHEELS */
      if(oi.gamePad.getRightButton()) {
        new SetIntakeRollers(1, true).start();
      } else if(oi.gamePad.getLeftButton()) {
        new SetIntakeRollers(1, false).start();
      }
      //*************************************************CARGO LIFT STATES */
      if(oi.gamePad.getButtonBPressed() && oi.gamePad.getRightTrigger() > 0.5) {
        new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_ONE, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.gamePad.getButtonYPressed() && oi.gamePad.getRightTrigger() > 0.5) {
        new MoveLiftCommand(LiftPositions.CARGO_ROCKET_LEVEL_TWO, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.gamePad.getButtonXPressed() && oi.gamePad.getRightTrigger() > 0.5) {
        new MoveLiftCommand(LiftPositions.CARGO_SHIP, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(oi.gamePad.getButtonAPressed() && oi.gamePad.getRightTrigger() > 0.5) {
        new MoveLiftCommand(LiftPositions.LOW, 2).start();
        SmartDashboard.putBoolean("Is Motion Magic", true);
      } else if(!lift.getIsMoving()) {
        lift.setManualLift(oi.gamePad.getLeftJoystickY()/2.0);
        SmartDashboard.putBoolean("Is Motion Magic", false);
      }


      /*****************************************************DRIVE */
      drive.driveFwdRotate(oi.driver.getDriverVertical(), oi.driver.getDriverHorizontal());
    }
}
