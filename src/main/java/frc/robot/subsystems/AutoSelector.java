package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class AutoSelector extends Subsystem {

    private DigitalInput side;
    private DigitalInput control;
    private Potentiometer rotary;

    // TODO: Write in the autos
    private final String[] autos = {"", "", "", "", "", ""}; //This is the actual filename so we can run the MP commands with this same pathName
    private final String[] autos_readable =  {"", "", "", "", "", ""}; //Readable version for smartdash and drivers --- should be same as autos[] in every other way
    private int chosenAuto;

    public AutoSelector() {
        side = new DigitalInput(0);
        control = new DigitalInput(1);
        rotary = new AnalogPotentiometer(2, autos.length, 0); //May need to change the offset value depending on pot shift
    }

    public boolean getControl() {
        return control.get(); //T-Auto, F-Teleop
    }

    public boolean getSide() {
        return side.get(); // T-Right, F-Left
    }

    public String getAuto() {
        int auto = (int) rotary.get();
        if(auto == 6) auto--;
        chosenAuto = auto;
        return autos[auto];
    }

    @Override
    protected void initDefaultCommand() {
        
    }
    @Override
    public void essentialShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("Side", side.get() ? "Right" : "Left");
        tab.add("Sandstorm Control", control.get() ? "Auto" : "Teleop");
        tab.add("Auto", autos[chosenAuto]);
    }
    @Override
    public void diagnosticShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("Auto", autos_readable[chosenAuto]);
    }
}
