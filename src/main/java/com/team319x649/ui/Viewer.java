package com.team319x649.ui;

import java.awt.FlowLayout;
import java.text.DecimalFormat;

import javax.swing.JFrame;
import javax.swing.JTabbedPane;

import com.team254.lib.trajectory.Path;
import com.team319x649.trajectory.FishyPath;

public class Viewer extends JFrame {

    private static final long serialVersionUID = 1L;
    JTabbedPane tabs;

    public Viewer() {
        setTitle("Paths");
        setLayout(new FlowLayout());
        setVisible(true);
        tabs = new JTabbedPane();
        tabs.setTabPlacement(JTabbedPane.LEFT);
        add(tabs);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }

    public void addPath(FishyPath fishyPath, Path path, double robotWidth, double robotHeight) {
        displayField(fishyPath, path, robotWidth, robotHeight);
        pack();
    }

    private String getPathNameAndTime(Path path) {
        DecimalFormat df = new DecimalFormat("0.00##");
        StringBuilder title = new StringBuilder();
		title.append(path.getName()).append(" : ")
		.append(df.format(path.getTrajectory().getNumSegments() * path.getTrajectory().getSegment(0).dt))
        .append("s");
        return title.toString();
    }

    private void displayField(FishyPath fishyPath, Path path, double robotW, double robotH) {
        tabs.addTab(getPathNameAndTime(path), new FieldComponent("field_image.png", fishyPath, path, robotW, robotH));
    }
}