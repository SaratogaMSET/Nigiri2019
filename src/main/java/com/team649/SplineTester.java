package com.team649;

import java.awt.Color;
import java.util.Timer;

import com.team254.lib.trajectory.Spline;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;

import frc.robot.util.FishyMath;

public class SplineTester {
    public static void main(String[] args) {
        Spline s = new Spline();
        double start_time = System.currentTimeMillis();
        Spline.reticulateSplines(new Waypoint(0, 3.2, FishyMath.d2r(15)), new Waypoint(10, 1.5, FishyMath.d2r(15)), s, Spline.CubicHermite);

        double x[] = new double[51];
        double y[] = new double[51];
        for(double i = 0.0; i <= 1.02; i += 0.01) {
            double[] xy = s.getXandY(i);
            x[(int) Math.round(50.0*i)] = xy[0];
            y[(int) Math.round(50.0*i)] = xy[1];
            // System.out.println(((int) 50*i)+": ("+xy[0]+", "+xy[1]+", " + 1.0/s.curvatureAt(i) + ")");
        }
        double end = System.currentTimeMillis();
        System.out.println("TOOK: " + (end - start_time) + " ms.");


        for(int i = 0; i <51; i++) {
            System.out.println("("+x[i]+", "+y[i]+")");
        }


        
    }
}