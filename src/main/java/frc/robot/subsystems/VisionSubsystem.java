package frc.robot.subsystems;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends Subsystem {
    @Override
    protected void initDefaultCommand() {
        
    }

    // Sensors
    public SerialPort jevoisSerial;
    public UsbCamera jevoisCamera;

    // State
    private String jevoisData;
    private Double angleDisplacement;

    // Comm
    private NetworkTable visionTable;

    public VisionSubsystem() throws Exception {
        // The vision module on Jevois runs at YUV 640x480 @ 30fps.
        // TODO: comment this out. Streaming from the Jevois camera is not needed for production/comp-level purposes, only for debugging.
        // TODO: Create a NONE output module for vision on the jevois.
        jevoisCamera = CameraServer.getInstance().startAutomaticCapture();
        jevoisCamera.setVideoMode(PixelFormat.kYUYV, 640, 480, 30);

        // To read angle data, open serial-over-USB port @ 115200 baud rate.
        jevoisSerial = new SerialPort(115200, SerialPort.Port.kUSB);

        // NT Table for logging
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    }

    public void readData() {
        NetworkTableEntry jevoisDataEntry = visionTable.getEntry("jevoisData");
        NetworkTableEntry jevoisAngleEntry = visionTable.getEntry("jevoisAngle");
        NetworkTableEntry angleEntry = visionTable.getEntry("angle");

        jevoisData = jevoisSerial.readString();
        jevoisDataEntry.setString(jevoisData);

        // Extract angle
        Pattern angleRegex = Pattern.compile(".*(ANGLE [-\\d.]+)");
        Matcher m = angleRegex.matcher(jevoisData);
        if(m.find()) {
            SmartDashboard.putNumber("FOUND VISION TARGET", 1);
            angleDisplacement = Double.parseDouble(m.group(1).substring(5));
        }
        else {
            SmartDashboard.delete("FOUND VISION TARGET");
        }
        if(angleDisplacement != null) {
            jevoisAngleEntry.setNumber(angleDisplacement);
            angleEntry.setNumber(angleDisplacement);
        }
        else {
            jevoisAngleEntry.delete();
        }
    }

    public Double getAngleDisplacement() {
        return angleDisplacement;
    }

    @Override
    public void diagnosticShuffleboard() {
        
    }

    @Override
    public void essentialShuffleboard() {
        
    }
}