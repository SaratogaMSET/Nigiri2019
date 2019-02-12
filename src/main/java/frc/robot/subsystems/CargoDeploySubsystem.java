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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class CargoDeploySubsystem extends Subsystem implements ILogger {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX intakeWheel;
  private DoubleSolenoid rightPiston;
  private DoubleSolenoid leftPiston;
  private DigitalInput cargoIR;

  private boolean running = false;
  private boolean angled = false;
  private boolean acquired = false;
  
  public CargoDeploySubsystem() {
    intakeWheel = new TalonSRX(RobotMap.CargoDeploy.bagIntake);
    cargoIR = new DigitalInput(RobotMap.CargoDeploy.cargoIRSensor);
    leftPiston = new DoubleSolenoid(RobotMap.CargoDeploy.INTAKE_SOL_LEFT[0], RobotMap.CargoDeploy.INTAKE_SOL_LEFT[1]);
    rightPiston = new DoubleSolenoid(RobotMap.CargoDeploy.INTAKE_SOL_RIGHT[0], RobotMap.CargoDeploy.INTAKE_SOL_RIGHT[1]);
  }

  public void runIntake(double power) {
    if(power != 0.0)
      running = true; 
    else 
      running = false;
    intakeWheel.set(ControlMode.PercentOutput, power);
  }

  public void anglePistons() {
    if(angled) {
      rightPiston.set(Value.kReverse);
      leftPiston.set(Value.kReverse);
      angled = false;
    }
    else {
      rightPiston.set(Value.kForward);
      leftPiston.set(Value.kForward);
      angled = true;
    }
  }

  public boolean hasCargo() {
    return acquired = cargoIR.get();
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
    cargoDeploy.add("Right Piston Shortage", rightPiston.getPCMSolenoidVoltageFault());
    cargoDeploy.add("Left Piston Shortage", leftPiston.getPCMSolenoidVoltageFault());
    cargoDeploy.add("Right Piston PCM Blacklist", rightPiston.getPCMSolenoidBlackList());
    cargoDeploy.add("Left Piston PCM Blacklist", leftPiston.getPCMSolenoidBlackList());
  }

  @Override
  public void essentialShuffleboard() {
    ShuffleboardTab cargoDeploy = Shuffleboard.getTab("Cargo");
    cargoDeploy.add("Cargo Acquired", acquired);
    cargoDeploy.add("Intake Running", running);
    cargoDeploy.add("Pistons Angled", angled);
  }
}
