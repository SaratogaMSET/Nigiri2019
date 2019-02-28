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

    // Const
    private static final Double CAMERA_OFFSET_FT = 0.5;

    // Sensors
    public SerialPort jevoisSerial;
    public UsbCamera jevoisCamera;

    // State
    private String jevoisData;
    private Double angleDisplacement;
    private Double distance;

    // Comm
    private NetworkTable visionTable;

    public VisionSubsystem() throws Exception {
        // The vision module on Jevois runs at YUV 640x480 @ 30fps.
        // TODO: comment this out. Streaming from the Jevois camera is not needed for production/comp-level purposes, only for debugging.
        // TODO: Create a NONE output module for vision on the jevois.
        jevoisCamera = CameraServer.getInstance().startAutomaticCapture(0);
        jevoisCamera.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);

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
        Pattern distRegex = Pattern.compile(".*(DISTANCE [-\\d.]+)");

        Matcher am = angleRegex.matcher(jevoisData);
        Matcher dm = distRegex.matcher(jevoisData);

        if(am.find()) {
            SmartDashboard.putNumber("VISION TARGET", 1);
            angleDisplacement = Double.parseDouble(am.group(1).substring(5));
            angleEntry.setNumber(angleDisplacement);
        }
        else {
            SmartDashboard.putNumber("VISION TARGET", 0);
            angleDisplacement = null;
        }
        if(dm.find()) {
            SmartDashboard.putNumber("VISION DIST", 1);
            distance = Double.parseDouble(dm.group(1).substring(8));
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

    public Double getOffsetCorrectedAngle() {
        if(angleDisplacement == null || distance == null) {
            return null;
        }
        Double d = Math.sqrt(distance*distance + CAMERA_OFFSET_FT*CAMERA_OFFSET_FT - 2 * CAMERA_OFFSET_FT * distance * Math.sin(Math.toRadians(angleDisplacement)));
        Double correctedAngle = Math.toDegrees(Math.asin(Math.cos(Math.toRadians(angleDisplacement)) * distance/d))-90;
        if(Math.abs(Math.sin(Math.toRadians(angleDisplacement)))*distance > CAMERA_OFFSET_FT) {
            correctedAngle = -correctedAngle;
        }
        SmartDashboard.putNumber("Corrected Angle", correctedAngle);
        return correctedAngle;
    }

    @Override
    public void diagnosticShuffleboard() {
        
    }

    @Override
    public void essentialShuffleboard() {
        
    }
}