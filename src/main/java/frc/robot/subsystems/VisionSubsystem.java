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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends Subsystem {
    @Override
    protected void initDefaultCommand() {
        
    }

    // Sensors
    public SerialPort jevoisSerial;

    // State
    private String jevoisData = "";

    public Double delta_x;
    public Double delta_y;
    public Double received_timestamp;

    private Double angleDisplacement;
    private Double distance;
    private Double timestamp_2;

    // Extract angle
    Pattern deltaXRegex = Pattern.compile(".*(DELTAX [-\\d.]+)");
    Pattern deltaYRegex = Pattern.compile(".*(DELTAY [-\\d.]+)");
    Pattern angleRegex = Pattern.compile(".*(ANGLE [-\\d.]+)");
    Pattern distRegex = Pattern.compile(".*(DISTANCE [-\\d.]+)");

    Matcher endMatcher;
    Matcher startMatcher;
    Matcher dxMatcher;
    Matcher dyMatcher;
    Matcher angleMatcher;
    Matcher distMatcher;

    // Comm
    private NetworkTable visionTable;

    public VisionSubsystem() throws Exception {
        // The vision module on Jevois runs at YUV 640x480 @ 30fps.
        // TODO: comment this out. Streaming from the Jevois camera is not needed for production/comp-level purposes, only for debugging.
        // TODO: Create a NONE output module for vision on the jevois.
        UsbCamera jevoisCamera = CameraServer.getInstance().startAutomaticCapture(0);
        jevoisCamera.setVideoMode(PixelFormat.kYUYV, 640, 480, 30);
        jevoisCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        // To read angle data, open serial-over-USB port @ 115200 baud rate.
        jevoisSerial = new SerialPort(115200, SerialPort.Port.kUSB);

        // NT Table for logging
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    }

    public void readData() {
        NetworkTableEntry deltaXEntry = visionTable.getEntry("Delta X");
        NetworkTableEntry deltaYEntry = visionTable.getEntry("Delta Y");
        String serial = jevoisSerial.readString();
        jevoisData += serial;

        // System.out.println("VISION LEN: " + jevoisData.length());
        // System.out.println("SERIAL LEN: " + serial.length());

        // System.out.println("SERIAL DATA: " + serial);
        // System.out.println("VISION DATA: " + jevoisData);


        int endIndex = jevoisData.lastIndexOf("END");
        if(endIndex > 0) {
            String splice = jevoisData.substring(0, endIndex);
            int startIndex = splice.lastIndexOf("START");
            if(startIndex > 0) {
                splice = splice.substring(startIndex);
                jevoisData = jevoisData.substring(endIndex+2);

                // System.out.println("JEVOIS SPLICE");
                // System.out.println(splice);
                // System.out.println("CLOSE SPLICE");

                if(dxMatcher == null) {
                    dxMatcher = deltaXRegex.matcher(splice);
                }
                else {
                    dxMatcher.reset(splice);
                }
                if(dyMatcher == null) {
                    dyMatcher = deltaYRegex.matcher(splice);
                }
                else {
                    dyMatcher.reset(splice);
                }
                if(angleMatcher == null) {
                    angleMatcher = angleRegex.matcher(splice);
                }
                else {
                    angleMatcher.reset(splice);
                }
                if(distMatcher == null) {
                    distMatcher = distRegex.matcher(splice);
                }
                else {
                    distMatcher.reset(splice);
                }

                if(dxMatcher.find() && dyMatcher.find()) {
                    double dx = Double.parseDouble(dxMatcher.group(1).substring(6)); 
                    double dy = Double.parseDouble(dyMatcher.group(1).substring(6)); 
    
                    delta_x = dx/12.0;
                    delta_y = dy/12.0;
                    received_timestamp = Timer.getFPGATimestamp();

                    deltaXEntry.setNumber(delta_x);
                    deltaYEntry.setNumber(delta_y);

                    // System.out.println("VISION MESSAGE:");
                    // System.out.println("DX: " + delta_x);
                    // System.out.println("DY: " + delta_y);
                    // System.out.println("TIME: " + received_timestamp);

                }

                if(distMatcher.find() && angleMatcher.find()) {
                    this.angleDisplacement = Double.parseDouble(angleMatcher.group(1).substring(5)); 
                    this.distance = Double.parseDouble(distMatcher.group(1).substring(8));
                    this.timestamp_2 = Timer.getFPGATimestamp();
                }
            }
        }

        if(jevoisData.length() > 1000) {
            jevoisData = "";
        }
    }

    @Override
    public void diagnosticShuffleboard() {
        
    }

    @Override
    public void essentialShuffleboard() {
        
    }

    public synchronized Double getDistance() {
        return distance;
    }

    public synchronized Double getAngle() {
        return angleDisplacement;
    }

    public synchronized Double getPIDTimestamp() {
        return this.timestamp_2;
    }
}