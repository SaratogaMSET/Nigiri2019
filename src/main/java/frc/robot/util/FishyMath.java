package frc.robot.util;

import frc.robot.subsystems.DrivetrainSubsystem;

public class FishyMath {
	public static double meters2feet(double meters) {
		return meters * 0.3048;
	}
	
	public static double feet2meters(double feet) {
		return feet * 3.28084;
	}
	
    public static double bound(double v, double low, double high) {
        return (v < low) ? low : ((v > high) ? high : v);
    }

    public static double bound(double v, double limit) {
        return bound(v, -limit, limit);
    }
    
    public static double bound(double input) {
        return bound(input, -1, 1);
    }
    
    public static double boundThetaNegPiToPi(double theta) {
        return theta - (Math.ceil((theta + Math.PI) / (Math.PI * 2)) - 1) * (Math.PI * 2); // (-π;π]
    }

    public static double boundTheta0To2Pi(double theta) {
        return theta - Math.floor(theta / (Math.PI * 2)) * (Math.PI * 2); // [0;2π)
    }

    public static double boundThetaNeg180to180(double theta) {
        return theta - (Math.ceil((theta + 180)/360)-1)*360; // (-180;180]
    }
    
    public static double boundTheta0to360(double theta) {
        return theta - Math.floor(theta/360)*360;  // [0;360)
    }
    
    public static double deltaThetaInDegrees(double from, double to) {
        return boundThetaNeg180to180(to - from);
    }

    public static double deltaThetaInRadians(double from, double to) {
        return boundThetaNegPiToPi(to - from);
    }
    
    public static int scale(int input,
            int lowInput, int highInput, int lowOutput, int highOutput)
    {
        final int inputRange = highInput - lowInput;
        final int outputRange = highOutput - lowOutput;
        
        return (input - lowInput) * outputRange / inputRange + lowOutput;
    }

    public static double scale(double input,
                               double lowInput, double highInput, double lowOutput, double highOutput)
    {
        final double inputRange = highInput - lowInput;
        final double outputRange = highOutput - lowOutput;
        
        return  (input - lowInput) * outputRange / inputRange + lowOutput;
    }

    public static double deadZone(double input, double deadband)
    {
        return Math.abs(input) >= deadband ? input : 0;
    }
    
    public static double rotations2feet(double rotations) {
        return rotations * DrivetrainSubsystem.WHEEL_DIAMETER * Math.PI;
    }
    
    public static double rpm2fps(double rpm) {
        // rpm * circumference will be feet / minute
        // 60 is the number of seconds in a minute
        return rpm * DrivetrainSubsystem.WHEEL_DIAMETER * Math.PI / 60.0;
    }
    
    public static double fps2rpm(double fps) {
        // fps * 60 will be feet / minute
        // feet / minute / circumference is rpm 
        return fps * 60.0 / (DrivetrainSubsystem.WHEEL_DIAMETER * Math.PI);
    }

    public static double mm2ft(double dist) {
        return dist / 304.8;
    }
    
    public static boolean isZero(double val) {
        return Math.abs(val) < 0.0000001;
    }
    
    public static boolean isEqual(double v1, double v2) {
        return isZero(v1 - v2);
    }

    public static double d2r(double degrees) {
        return degrees * Math.PI / 180.0;
    }

    public static double r2d(double rad) {
        return rad * 180.0 / Math.PI;
    }

    public static double rpm2talonunits(double rpm) {
        return rpm * ((double) DrivetrainSubsystem.TICKS_PER_REV) / 600.0;
    }

    public static double talaonunits2rpm(double talonunits) {
        return talonunits * 600.0 / ((double)DrivetrainSubsystem.TICKS_PER_REV);
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}