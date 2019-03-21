package com.team649;

import java.awt.Color;
import java.awt.geom.Ellipse2D;
import java.io.IOException;
import java.util.Arrays;

import com.team254.lib.trajectory.Spline;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;

import frc.robot.util.FishyMath;

import java.awt.Color;
import javax.swing.JFrame;
import de.erichseifert.gral.data.DataTable;
import de.erichseifert.gral.graphics.Insets2D;
import de.erichseifert.gral.plots.XYPlot;
import de.erichseifert.gral.plots.lines.DefaultLineRenderer2D;
import de.erichseifert.gral.plots.lines.LineRenderer;
import de.erichseifert.gral.plots.points.DefaultPointRenderer2D;
import de.erichseifert.gral.plots.points.PointRenderer;
import de.erichseifert.gral.ui.InteractivePanel;

public class PurePursuitVisualization {
    public static class Plot extends JFrame {
        public DataTable data;
        public DataTable ext;
        public Plot(Double[] x, Double[] y) {
            setDefaultCloseOperation(EXIT_ON_CLOSE);
            setSize(600, 400);
    
            data = new DataTable(Double.class, Double.class);
            for (int i = 0; i < Math.min(x.length, y.length); i++) {
                data.add(x[i], y[i]);
            }
            ext = new DataTable(Double.class, Double.class);
            XYPlot plot = new XYPlot(data, ext);
            getContentPane().add(new InteractivePanel(plot));
            
            double insetsTop = 20.0,
                    insetsLeft = 60.0,
                    insetsBottom = 60.0,
                    insetsRight = 40.0;
            plot.setInsets(new Insets2D.Double(insetsTop, insetsLeft, insetsBottom, insetsRight));

            PointRenderer splinePointsRender = new DefaultPointRenderer2D();
            splinePointsRender.setShape(new Ellipse2D.Double(-3.0, -3.0, 6.0, 6.0));
            splinePointsRender.setColor(Color.BLACK);
            plot.setPointRenderers(data, splinePointsRender);

            PointRenderer extPointsRender = new DefaultPointRenderer2D();
            extPointsRender.setShape(new Ellipse2D.Double(-5.0, -5.0, 10.0, 10.0));
            extPointsRender.setColor(Color.green);
            plot.setPointRenderers(ext, extPointsRender);

            LineRenderer lines = new DefaultLineRenderer2D();
            lines.setColor(Color.BLUE);
            plot.setLineRenderers(data, lines);


            plot.getAxisRenderer(XYPlot.AXIS_X).setIntersection(-Double.MAX_VALUE);
            plot.getAxisRenderer(XYPlot.AXIS_Y).setIntersection(-Double.MAX_VALUE);

            

            setVisible(true);
        }
    }

    public static double ROBOT_X = 6.0;
    public static double ROBOT_Y = 0.3;
    public static double ROBOT_HEADING = FishyMath.d2r(35);

    public static int LAST_CLOSEST_POINT = 40;

    public static Spline path;
    public static Double spline_x[];
    public static Double spline_y[];


    // CONSTANTS
    public static int NUM_SAMPLES = 100;
    public static double LOOKAHEAD_DISTANCE = 2.5; // feet



    public static double[] getGoalPoint() {
    // (x - robotx)^2 + (y - roboty)^2 = l^2
        double[] goal = new double[3];
        double mindist = Double.POSITIVE_INFINITY;
        for(int i = LAST_CLOSEST_POINT; i < NUM_SAMPLES; i++) {
            double d = LOOKAHEAD_DISTANCE - Math.hypot(spline_x[i] - ROBOT_X, spline_y[i] - ROBOT_Y);
            if(d > 0.0 && d <= mindist) {
                mindist = d;
                goal = new double[] {spline_x[i], spline_y[i], -d + LOOKAHEAD_DISTANCE};
                LAST_CLOSEST_POINT = i;
            }
        }
        return goal;
    }

    public static double getCurvature(double[] goal) {
        double a = -Math.tan(ROBOT_HEADING);
        double c = Math.tan(ROBOT_HEADING) * ROBOT_X - ROBOT_Y;
        double x = Math.abs(a * goal[0] + goal[1] + c)/Math.sqrt(a*a + 1);
        double curvature = 2 * x / (goal[2] * goal[2]);
        double side = Math.signum(Math.sin(ROBOT_HEADING) * (goal[0] - ROBOT_X) - Math.cos(ROBOT_HEADING) * (goal[1] - ROBOT_Y));
        return side * curvature;
    }

    public static double getLeftToRightVelocityRatio(double curvature, double wheelbase) {
        return (2.0 + curvature * wheelbase)/(2.0 - curvature * wheelbase);
    }

    public static void main(String[] args) {
        path = new Spline();
        Spline.reticulateSplines(new Waypoint(0, 0, FishyMath.d2r(15)), new Waypoint(16, 3.7, FishyMath.d2r(0)), path, Spline.QuinticHermite);
        spline_x = new Double[NUM_SAMPLES+1];
        spline_y = new Double[NUM_SAMPLES+1];
        for (double i = 0.0; i <= (1.0 + 1.0/(double) NUM_SAMPLES); i += 1.0/(double) NUM_SAMPLES) {
            double[] xy = path.getXandY(i);
            spline_x[(int) Math.round((double) NUM_SAMPLES * i)] = xy[0];
            spline_y[(int) Math.round((double) NUM_SAMPLES * i)] = xy[1];
            // System.out.println(path.calculateLength(i));
            // System.out.println(path.calculateLength());
        }

        Plot p = new Plot(spline_x, spline_y);
        double[] gp = getGoalPoint();
        p.ext.add(gp[0], gp[1]);
        p.ext.add(ROBOT_X, ROBOT_Y);
        p.ext.add(ROBOT_X + Math.cos(ROBOT_HEADING), ROBOT_Y + Math.sin(ROBOT_HEADING));
        System.out.println(getLeftToRightVelocityRatio(getCurvature(gp), 2.3));

    }
}