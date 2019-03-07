package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.BlockingQueue;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
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
    private Double distance;

    public ArrayList<Double> angleHistory; // can contain null
    private static final int ANGLE_HISTORY_LENGTH = 300;

    // Comm
    private NetworkTable visionTable;

    public VisionSubsystem() throws Exception {
        // The vision module on Jevois runs at YUV 640x480 @ 30fps.
        // TODO: comment this out. Streaming from the Jevois camera is not needed for production/comp-level purposes, only for debugging.
        // TODO: Create a NONE output module for vision on the jevois.
        jevoisCamera = CameraServer.getInstance().startAutomaticCapture("Jevois", 0);
        jevoisCamera.setVideoMode(PixelFormat.kYUYV, 640, 480, 30);
        jevoisCamera.setConnectVerbose(1);
        jevoisCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        
        // To read angle data, open serial-over-USB port @ 115200 baud rate.
        jevoisSerial = new SerialPort(115200, SerialPort.Port.kUSB);

        // NT Table for logging
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        angleHistory = new ArrayList<>();
    }

    public void readData() {
        NetworkTableEntry jevoisDataEntry = visionTable.getEntry("jevoisData");
        NetworkTableEntry jevoisAngleEntry = visionTable.getEntry("jevoisAngle");
        NetworkTableEntry angleEntry = visionTable.getEntry("angle");

        jevoisData += jevoisSerial.readString();
        jevoisDataEntry.setString(jevoisData);
        if(jevoisData.length() > 500) {
            jevoisData = "";
        }

        // Extract angle
        Pattern angleRegex = Pattern.compile(".*(ANGLE [-\\d.]+)");
        Pattern distRegex = Pattern.compile(".*(DISTANCE [-\\d.]+)");

        Matcher am = angleRegex.matcher(jevoisData);

        if(am.find()) {
            SmartDashboard.putNumber("VISION TARGET", 1);
            angleDisplacement = Double.parseDouble(am.group(1).substring(5));
            if(Math.abs(angleDisplacement) > 30.0) {
                angleDisplacement = null;
            }
            else {
                angleEntry.setNumber(angleDisplacement);
                SmartDashboard.putNumber("VISION ANGLE", angleDisplacement);
            }
            angleHistory.add(angleDisplacement);
            if(angleHistory.size() > ANGLE_HISTORY_LENGTH) {
                angleHistory.remove(0);
            }
            jevoisData = jevoisData.substring(am.start(1));
        }
        else {
            SmartDashboard.putNumber("VISION TARGET", 0);
            angleDisplacement = null;
        }
        Matcher dm = distRegex.matcher(jevoisData);
        if(dm.find()) {
            SmartDashboard.putNumber("VISION DIST", 1);
            distance = Double.parseDouble(dm.group(1).substring(8));
            jevoisData = jevoisData.substring(dm.start(1));
        }
        else {
            SmartDashboard.putNumber("VISION DIST", 0);
            distance = null;
        }
    }

    public Double getAngleDisplacement() {
        return angleDisplacement;
    }

    public Double getDistance() {
        return distance;
    }

    @Override
    public void diagnosticShuffleboard() {
        
    }

    @Override
    public void essentialShuffleboard() {
        
    }
}