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
		public static double[] logTimer, PIDOutput, Oscillations, OscillationTimes;
		public static Map<Double, String> commands = new HashMap<Double, String>();
		String inputLine;
		public static String inFileRobotParam = "/home/admin/param.txt", outFileLog = "/home/admin/logfile.csv";
		public static PrintWriter writer;
		public static PrintWriter commandWriter;
		static double[] armCurrent;
		static double[] armPower;
		static double[] liftPower;
		static double[] liftCurrent;
		static String[] commandsArray;
		static Timer timer;
		public static int index = 0;
	// ************************************************************************	
	
		public Logging() {
			timer = new Timer();
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

		public static void closeWriter() {
			commandWriter.close();
		}
		
		public static void createLogFile() {
			try {
				// outFileLog = new SimpleDateFormat("'/home/lvuser/logfile'.MMddhhmm'.csv'").format(new Date());
				// System.out.printf("outputfile is: %s\n",  outFileLog);
			
				// File file = new File(outFileLog);
				// writer = new PrintWriter(outFileLog, "UTF-8");

				String commandFileLocation = new SimpleDateFormat("'/home/lvuser/commands'.MMddhhmm'.csv'").format(new Date());
				File commandFile = new File(commandFileLocation);
				commandWriter = new PrintWriter(commandFileLocation, "UTF-8");
				index = 0;
			} catch (FileNotFoundException | UnsupportedEncodingException e) {
				debugMode = false;
			}
		} 
		public static void writeToArray() {
			if (tick < maxTick) {
				// armCurrent[tick] = Robot.arm.bottomMotor.getOutputCurrent();
				// armPower[tick] = Robot.arm.bottomMotor.getMotorOutputVoltage();
				// liftCurrent[tick] = Robot.lift.mainLiftMotor.getOutputCurrent();
				// liftPower[tick] = Robot.lift.mainLiftMotor.getMotorOutputVoltage();
				tick++;
			} // end maxTick check
		}

		public static void print(String value) {
			commandWriter.printf("%s%n", value);
		}
}
