package com.team319x649.trajectory;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.io.TextFileSerializer;

public class FishyPathGenerator extends PathGenerator {

	public static Path makePath(FishyPath fishyPath) {
		Path p = new Path();

		p = PathGenerator.makePath(fishyPath.getWaypointSequence(), fishyPath.getConfig(),
		fishyPath.getWheelbaseFeet(), fishyPath.getName());

		if (!isDirectionValid(fishyPath)) {
			p.offsetHeading(-Math.PI);
		}

		if (fishyPath.isBackwardsPath()) {
			p = reversePath(p);
		}
		
		return p;
	}

	private static boolean isDirectionValid(FishyPath path) {
		WaypointSequence wps = path.getWaypointSequence();
		if (wps.getWaypoint(wps.getNumWaypoints() - 1).x >= wps.getWaypoint(0).x) {
			return true;
		} else {
			return false;
		}
	}

	private static Path reversePath(Path p) {
		Trajectory.Pair pair = p.getTrajectoryPair();
		pair.left.scale(-1);
		pair.right.scale(-1);
		pair.center.scale(-1);
		return new Path(p.getName(), pair);
	}

	public static boolean exportPathToTextFile(String filePathPrefix, Path path) {
		String filePathLeft = filePathPrefix + "-left.csv";
		String filePathCenter = filePathPrefix + "-center.csv";
		String filePathRight = filePathPrefix + "-right.csv";
		try {
		  File fileL = new File(filePathLeft);
		  File fileR = new File(filePathRight);
		  File fileC = new File(filePathCenter);

		  // if file doesnt exists, then create it
		  if (!fileL.exists()) {
			fileL.createNewFile();
		  }
		  if (!fileR.exists()) {
			fileR.createNewFile();
		  }
		  if (!fileC.exists()) {
			fileC.createNewFile();
		  }

		  TextFileSerializer js = new TextFileSerializer();
		  String[] serialized = js.serialize(path);

		  FileWriter fw = new FileWriter(fileL);
		  BufferedWriter bw = new BufferedWriter(fw);
		  bw.write(serialized[0]);
		  bw.flush();
		  bw.close();

		  fw = new FileWriter(fileC);
		  bw = new BufferedWriter(fw);
		  bw.write(serialized[1]);
		  bw.flush();
		  bw.close();

		  fw = new FileWriter(fileR);
		  bw = new BufferedWriter(fw);
		  bw.write(serialized[2]);
		  bw.flush();
		  bw.close();


		} catch (IOException e) {
			e.printStackTrace();
			return false;
		}
		
		return true;
	  }

	public static void copyFilesToRelativeDirectory(String fromDirectoryRelative, String toDirectoryRelative) {
		int fileCount = 0;
		File fromDirectory = new File(fromDirectoryRelative);
		File toDirectory = new File(toDirectoryRelative);

		for (File source : fromDirectory.listFiles()) {
			try {
				fileCount++;
				File dest = new File(toDirectory + "\\" + source.getName());
				copyFileUsingStream(source, dest);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		System.out.println("Copied " + fileCount + " files to " + toDirectoryRelative);
	}

	private static void copyFileUsingStream(File source, File dest) throws IOException {
		InputStream is = null;
		OutputStream os = null;
		try {
			is = new FileInputStream(source);
			os = new FileOutputStream(dest);
			byte[] buffer = new byte[1024];
			int length;
			while ((length = is.read(buffer)) > 0) {
				os.write(buffer, 0, length);
			}
		} finally {
			is.close();
			os.close();
		}
	}
}