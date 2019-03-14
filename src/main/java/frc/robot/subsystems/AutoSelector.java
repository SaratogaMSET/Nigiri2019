package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector extends Subsystem {

    private DigitalInput side;
    private DigitalInput control;
    private Potentiometer rotary;

    // TODO: Write in the autos
    private final String[] autos = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10"}; //This is the actual filename so we can run the MP commands with this same pathName
    private final String[] autos_readable = { "1r", "2r", "3r", "4r", "5r", "6r", "7r", "8r", "9r", "10r" }; //Readable version for smartdash and drivers --- should be same as autos[] in every other way
    private int chosenAuto;

    public AutoSelector() {
        side = new DigitalInput(RobotMap.AutoSelector.SIDE);
        control = new DigitalInput(RobotMap.AutoSelector.CONTROL);
        rotary = new AnalogPotentiometer(RobotMap.AutoSelector.ROTARY, autos.length, 0); //May need to change the offset value depending on pot shift
    }

    public String getControl() {
        return control.get() ? "Auto" : "Teleop"; //T-Auto, F-Teleop
    }

    public String getSide() {
        return side.get() ? "Left" : "Right"; // T-Left, F-Right
    }

    public String getAuto() {
        int auto = (int) rotary.get();
        if(auto == 10) auto--;
        chosenAuto = auto;
        return autos[auto];
    }

    @Override
    protected void initDefaultCommand() {

    }
    @Override
    public void essentialShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("Side", getSide());
        tab.add("Sandstorm Control", getControl());
        tab.add("Auto", autos[chosenAuto]);
    }
    @Override
    public void diagnosticShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("Auto", autos_readable[chosenAuto]);
    }
}
