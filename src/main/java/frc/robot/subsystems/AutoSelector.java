package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

public class AutoSelector extends Subsystem {

    AnalogInput selector1;
    AnalogInput selector2;

    public AutoSelector() {
        selector1 = new AnalogInput(0);
        selector2 = new AnalogInput(1);
        setup();
    }

    private void setup() {
        selector1.setOversampleBits(4);
        selector1.setAverageBits(2);
        selector2.setOversampleBits(4);
        selector2.setAverageBits(2);
    }

    public String getAuto() {
        
        return null;
    }

    @Override
    protected void initDefaultCommand() {
        
    }
    @Override
    public void essentialShuffleboard() {
        
    }
    @Override
    public void diagnosticShuffleboard() {
        
    }
}