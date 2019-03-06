package com.team319x649.ui;

import javax.swing.SwingUtilities;

import com.team254.lib.trajectory.Path;
import com.team319x649.*;
import com.team319x649.trajectory.FishyPath;

public class PathViewer {

	private static Viewer viewer = new Viewer();

	public static void showPath(FishyPath fishyPath, Path path) {
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				viewer.addPath(fishyPath, path, FishyPathCreator.robotWidthIn, FishyPathCreator.robotLengthIn);
			}
		});
	}

}
