/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class CargoIntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX leftIntake, rightIntake, backIntake;
  public DoubleSolenoid intakeSol;
  boolean isOut;

  public CargoIntakeSubsystem(){

    leftIntake = new TalonSRX(RobotMap.CargoIntake.LEFT_INTAKE);
    rightIntake = new TalonSRX(RobotMap.CargoIntake.RIGHT_INTAKE);
    backIntake = new TalonSRX(RobotMap.CargoIntake.BACK_INTAKE);

    intakeSol = new DoubleSolenoid(RobotMap.CargoIntake.INTAKE_SOL[0],RobotMap.CargoIntake.INTAKE_SOL[1],RobotMap.CargoIntake.INTAKE_SOL[2]);

    leftIntake.configNominalOutputForward(0, Robot.timeoutMs);
    leftIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    leftIntake.configPeakOutputForward(1, Robot.timeoutMs);
    leftIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    rightIntake.configNominalOutputForward(0, Robot.timeoutMs);
    rightIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    rightIntake.configPeakOutputForward(1, Robot.timeoutMs);
    rightIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    backIntake.configNominalOutputForward(0, Robot.timeoutMs);
    backIntake.configNominalOutputReverse(0, Robot.timeoutMs);
    backIntake.configPeakOutputForward(1, Robot.timeoutMs);
    backIntake.configPeakOutputReverse(-1, Robot.timeoutMs);

    isOut = false;
  }

  public void testMotor(int motor, double power){
    switch(motor){
      case 1:
        leftIntake.set(ControlMode.PercentOutput, power);
        break;
      case 2:
        rightIntake.set(ControlMode.PercentOutput, power);
        break;
      case 3:
        backIntake.set(ControlMode.PercentOutput, power);
        break;
      
    }
  }

  public void switchSol(){
    if(isOut){
      intakeSol.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      intakeSol.set(DoubleSolenoid.Value.kForward);
    }
    isOut = !isOut;

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
