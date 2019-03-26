package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector extends Subsystem {
    public static enum Side {
        LEFT, RIGHT
    }

    public static enum Control {
        AUTO, TELEOP
    }

    public static class PotValues {
        public static final double[] range1 = {0.007, 0.01};
        public static final double[] range2 = {0.535, 0.54};
        public static final double[] range3 = {1.05, 1.06};
        public static final double[] range4 = {1.55, 1.6};
        public static final double[] range5 = {2.06, 2.1};
        public static final double[] range6 = {2.5, 2.6};
        public static final double[] range7 = {3.1, 3.2};
        public static final double[] range8 = {3.6, 3.7};
        public static final double[] range9 = {4.2, 4.3};
        public static final double[] range10 = {4.8, 5};
    }
    private DigitalInput side;
    private DigitalInput control;
    private AnalogInput rotary;

    // TODO: Write in the autos
    private final String[] autos = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10"}; //This is the actual filename so we can run the MP commands with this same pathName
    private final String[] autos_readable = { "1r", "2r", "3r", "4r", "5r", "6r", "7r", "8r", "9r", "10r" }; //Readable version for smartdash and drivers --- should be same as autos[] in every other way
    private int chosenAuto;

    public AutoSelector() {
        side = new DigitalInput(RobotMap.AutoSelector.SIDE);
        control = new DigitalInput(RobotMap.AutoSelector.CONTROL);
        rotary = new AnalogInput(RobotMap.AutoSelector.ROTARY); //May need to change the offset value depending on pot shift
    }

    public Control getControl() {
        return control.get() ? Control.TELEOP : Control.AUTO; //T-Auto, F-Teleop
    }

    public Side getSide() {
        return side.get() ? Side.RIGHT : Side.LEFT; // T-Left, F-Right
    }

    // public String getAuto() {
    //     // int auto = (int) rotary.get();
    //     // if(auto == 10) auto--;
    //     // chosenAuto = auto;
    //     // return autos[auto];
    // }

    public double getPotVoltage() {
        return rotary.getVoltage();
    }

    public int getAutoPotNumber() {
        double val = getPotVoltage();
        if(inRange(val, PotValues.range1[0], PotValues.range1[1])) {
            return 1;
        } else if(inRange(val, PotValues.range2[0], PotValues.range2[1])) {
            return 2;
        } else if(inRange(val, PotValues.range3[0], PotValues.range3[1])) {
            return 3;
        } else if(inRange(val, PotValues.range4[0], PotValues.range4[1])) {
            return 4;
        } else if(inRange(val, PotValues.range5[0], PotValues.range5[1])) {
            return 5;
        } else if(inRange(val, PotValues.range6[0], PotValues.range6[1])) {
            return 6;
        } else if(inRange(val, PotValues.range7[0], PotValues.range7[1])) {
            return 7;
        } else if(inRange(val, PotValues.range8[0], PotValues.range8[1])) {
            return 8;
        } else if(inRange(val, PotValues.range9[0], PotValues.range9[1])) {
            return 9;
        } else if(inRange(val, PotValues.range10[0], PotValues.range10[1])) {
            return 10;
        } else {
            return 0;
        }
    }
    public boolean inRange(double x, double lower, double upper) {
        if (x > lower && x < upper) {
            return true;
        }
        return false;
    }

    @Override
    protected void initDefaultCommand() {

    }
    @Override
    public void essentialShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("Side", getSide() == Side.LEFT ? "Left" : "Right");
        tab.add("Sandstorm Control", getControl() == Control.TELEOP ? "Teleop" : "Auto");
        tab.add("Auto", autos[chosenAuto]);
    }
    @Override
    public void diagnosticShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("Auto", autos_readable[chosenAuto]);
    }
}
