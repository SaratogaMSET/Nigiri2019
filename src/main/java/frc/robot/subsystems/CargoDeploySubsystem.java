/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotState;

/**
 * Add your docs here.
 */
public class CargoDeploySubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static enum CargoDeployMotorState {
    INTAKE,
    EXTAKE,
    NONE
  }

  public static enum CargoGamePiece {
    NO_CARGO,
    HAVE_CARGO
  }

  private TalonSRX intakeWheel;
  private Solenoid piston;
  private DigitalInput cargoIR;

  private boolean running = false;
  private boolean angled = false;
  private boolean acquired = false;
  
  public CargoDeploySubsystem() {
    intakeWheel = new TalonSRX(RobotMap.CargoDeploy.DEPLOY_INTAKE_MOTOR);

    intakeWheel.configContinuousCurrentLimit(40, Robot.timeoutMs);
    intakeWheel.configPeakCurrentDuration(1500, Robot.timeoutMs);
    intakeWheel.configPeakCurrentLimit(40, Robot.timeoutMs);
    intakeWheel.enableCurrentLimit(true);
    
    cargoIR = new DigitalInput(RobotMap.CargoDeploy.IR_SENSOR);
    //piston = new Solenoid(4, RobotMap.CargoDeploy.INTAKE_SOL);
  }

  public void runIntake(double power) {
    if(power != 0.0)
      running = true; 
    else 
      running = false;
    intakeWheel.set(ControlMode.PercentOutput, -power);
  }

  public void anglePistons() {
    if(angled) {
      piston.set(false);
      angled = false;
    }
    else {
      piston.set(true);
      angled = true;
    }
  }

  public boolean hasCargo() {
    return !cargoIR.get();
  }

  public CargoGamePiece updateCargoGamePieceState() {
    if(hasCargo()) {
      return CargoGamePiece.HAVE_CARGO;
    } else {
      return CargoGamePiece.NO_CARGO;
    }
  }

  public CargoDeployMotorState updateCargoDeployState() {
    double percentOutput = intakeWheel.getMotorOutputPercent();
    if(percentOutput < 0) {
      return CargoDeployMotorState.INTAKE;
    } else if(percentOutput > 0) {
      return CargoDeployMotorState.EXTAKE;
    } else {
      return CargoDeployMotorState.NONE;
    }
  }

  public double getPercentOutput() {
    return intakeWheel.getMotorOutputPercent();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void diagnosticShuffleboard() {
    ShuffleboardTab cargoDeploy = Shuffleboard.getTab("Cargo");
    cargoDeploy.add("Cargo Acquired", acquired);

    cargoDeploy.add("Intake Voltage", intakeWheel.getMotorOutputVoltage());
    cargoDeploy.add("Intake Percentage", intakeWheel.getMotorOutputPercent());
    cargoDeploy.add("Intake Running", running);

    cargoDeploy.add("Angled", angled);
    
  }

  @Override
  public void essentialShuffleboard() {
    ShuffleboardTab cargoDeploy = Shuffleboard.getTab("Cargo");
    cargoDeploy.add("Cargo Acquired", acquired);
    cargoDeploy.add("Intake Running", running);
    cargoDeploy.add("Pistons Angled", angled);
  }

  @Override
  public void stopAll() {
    runIntake(0.0);
  }
}