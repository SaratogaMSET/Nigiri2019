/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

/**
 * NOT BEING USED
 */
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.wpilibj.Timer;

public class Logging {
	// for logging to file and reading parameters from file ********************
		public static boolean debugMode = true; // set this to true to enable
		public static final int maxTick = 15000; // at 50 samples/s need 3000 for 1
												// minute
		public static int tick;
		public static int value;
		public static double[] logTimer, PIDInput, PIDOutput, Oscillations, OscillationTimes;
		Map<String, Float> mapRobotParams = new HashMap<String, Float>();
		String inputLine;
		String[] inputWords = new String[100];
		public static String inFileRobotParam = "/home/admin/param.txt", outFileLog = "/home/admin/logfile.csv";
		public static PrintWriter writer;
		static double[] armCurrent;
		static double[] armPower;
		static double[] liftPower;
		static double[] liftCurrent;
		static Timer timer;
	// ************************************************************************	
	
		public Logging() {
			// armCurrent = new double[15000];
			// armPower = new double[15000];
			// liftPower = new double[15000];
			// liftCurrent = new double[15000];
			timer = new Timer();
			tick = 0;
		}
		public static void writeToFile() {
			for (int i = 0; i < tick; i++) {
				writer.printf("%d, %f", i, logTimer[i]);
				writer.printf(", %f, %f %f, %f %n", armCurrent[i], armPower[i], 
						liftCurrent[i], liftPower[i]);
			}
			tick = 0;
			writer.close();
		}
		
		public static void createLogFile() {
			try {
				outFileLog = new SimpleDateFormat("'/home/lvuser/logfile'.MMddhhmm'.csv'").format(new Date());
				System.out.printf("outputfile is: %s\n",  outFileLog);
			
				File file = new File(outFileLog);
				writer = new PrintWriter(outFileLog, "UTF-8");
			} catch (FileNotFoundException | UnsupportedEncodingException e) {
				debugMode = false;
			}
		} 
		public static void writeToArray() {
			if (tick < maxTick) {
				logTimer[tick] = timer.get();
				// armCurrent[tick] = Robot.arm.bottomMotor.getOutputCurrent();
				// armPower[tick] = Robot.arm.bottomMotor.getMotorOutputVoltage();
				// liftCurrent[tick] = Robot.lift.mainLiftMotor.getOutputCurrent();
				// liftPower[tick] = Robot.lift.mainLiftMotor.getMotorOutputVoltage();
				tick++;
			} // end maxTick check
		}
}
