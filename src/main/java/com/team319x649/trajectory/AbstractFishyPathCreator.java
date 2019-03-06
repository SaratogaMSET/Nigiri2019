
package com.team319x649.trajectory;

import java.util.List;

import com.team254.lib.trajectory.Path;
import com.team319x649.ui.PathViewer;
import com.team319x649.ui.Viewer;

/**
 * Forked from 254's 2014 Trajectory library just a comment to make a change
 * 
 * @author Jared341
 * @author ttremblay
 */
public abstract class AbstractFishyPathCreator {

	/**
	 * Generate the path files, to include config paths, display the paths in GUIS, and 
	 * move the files into the robot code project
	 */
	protected void generatePaths() {
		generatePathFiles(getPaths());
	}

	/**
	 * Return the list of arcs that are to be generated
	 */
	protected abstract List<FishyPath> getPaths(); 

	private void generatePathFiles(List<FishyPath> paths) {
		for (FishyPath path : paths) {
			Path p = FishyPathGenerator.makePath(path);
			FishyPathGenerator.exportPathToTextFile("src/main/java/frc/paths/" + path.getName(), p);
			PathViewer.showPath(path, p);
		}
	}
}
