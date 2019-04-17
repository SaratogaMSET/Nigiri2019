package frc.robot.util.pid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.util.FishyMath;

public class TalonPIDSetter extends SendableBase {
    private TalonSRX talon;
    double kf, kp, ki, kd, izone;

    public TalonPIDSetter(TalonSRX talon) {
        this.talon = talon;
    }

    public void setFF(double kf) {
        this.kf = kf;
        talon.config_kF(0, kf, 200);
    }

    public void setP(double p) {
        kp = p;
        talon.config_kP(0, kp, 200);
    }

    public void setI(double i) {
        ki = i;
        talon.config_kI(0, ki, 200);
    }

    public void setD(double d) {
        kd = d;
        talon.config_kD(0, kd, 200);
    }

    public void setIZone(double iz) {
        izone = iz;
        talon.config_IntegralZone(0, (int) (FishyMath.rpm2talonunits(FishyMath.fps2rpm(izone))), 200);
    }

    public double getFF() {
        return kf;
    }

    public double getP() {
        return kp;
    }

    public double getI() {
        return ki;
    }

    public double getD() {
        return kd;
    }

    public double getIZone() {
        return izone;
    }

    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kF", this::getFF, this::setFF);
        builder.addDoubleProperty("kP", this::getP, this::setP);
        builder.addDoubleProperty("kI", this::getI, this::setI);
        builder.addDoubleProperty("kD", this::getD, this::setD);
        builder.addDoubleProperty("I Zone", this::getIZone, this::setIZone);
    }
}