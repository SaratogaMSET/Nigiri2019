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
    private String jevoisData;

    public Double delta_x;
    public Double delta_y;
    public Double received_timestamp;

    // Extract angle
    Pattern endRegex = Pattern.compile(".*(END)");
    Pattern startRegex = Pattern.compile(".*(START)");
    Pattern deltaXRegex = Pattern.compile(".*(DELTAX [-\\d.]+)");
    Pattern deltaYRegex = Pattern.compile(".*(DELTAY [-\\d.]+)");


    // Comm
    private NetworkTable visionTable;

    public VisionSubsystem() throws Exception {
        // The vision module on Jevois runs at YUV 640x480 @ 30fps.
        // TODO: comment this out. Streaming from the Jevois camera is not needed for production/comp-level purposes, only for debugging.
        // TODO: Create a NONE output module for vision on the jevois.
        UsbCamera jevoisCamera = CameraServer.getInstance().startAutomaticCapture(0);
        jevoisCamera.setVideoMode(PixelFormat.kYUYV, 640, 480, 30);
        jevoisCamera.setConnectVerbose(1);
        jevoisCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        
        // To read angle data, open serial-over-USB port @ 115200 baud rate.
        jevoisSerial = new SerialPort(115200, SerialPort.Port.kUSB);

        // NT Table for logging
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    }

    public void readData() {
        NetworkTableEntry deltaXEntry = visionTable.getEntry("Delta X");
        NetworkTableEntry deltaYEntry = visionTable.getEntry("Delta Y");

        jevoisData += jevoisSerial.readString();
        if(jevoisData.length() > 500) {
            jevoisData = jevoisData.substring(100);
        }

        // System.out.println(jevoisData);

        Matcher endMatcher = endRegex.matcher(jevoisData);

        if(endMatcher.find()) {
            int substrIndex = endMatcher.start(1);
            String processData = jevoisData.substring(0, substrIndex);

            System.out.println("DATA:" + processData);
            Matcher startMatcher = startRegex.matcher(processData);
            if(startMatcher.find()) {
                int startMatch = startMatcher.start(1);

                processData = processData.substring(startMatch);
    
                Matcher dxMatcher = deltaXRegex.matcher(processData);
                Matcher dyMatcher = deltaYRegex.matcher(processData);

                if(dxMatcher.find() && dyMatcher.find()) {
                    double dx = Double.parseDouble(dxMatcher.group(1).substring(6)); 
                    double dy = Double.parseDouble(dyMatcher.group(1).substring(6)); 
        
                    delta_x = dx/12.0;
                    delta_y = dy/12.0;
                    received_timestamp = Timer.getFPGATimestamp();

                    System.out.println("VISION MESSAGE:");
                    System.out.println("DX: " + delta_x);
                    System.out.println("DY: " + delta_y);
                    System.out.println("TIME: " + received_timestamp);

                }
            }
        }
    }

    @Override
    public void diagnosticShuffleboard() {
        
    }

    @Override
    public void essentialShuffleboard() {
        
    }
}