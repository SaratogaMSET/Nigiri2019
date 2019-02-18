package frc.robot.safety;

import frc.robot.Robot;

/**
 * Example safety check. Ensures that the robot doesn't move.
 */
public class ExampleSafetyCheck {
    /**
     * The safety condition.
     * @return boolean true if the robot is performing safe action, false otherwise
     */
    public static boolean isSafe() {
        return Robot.drive.motors[0].getMotorOutputPercent() == 0.0 && Robot.drive.motors[3].getMotorOutputPercent() == 0.0;
    }

    public static boolean ensureSafety() {
        if(!isSafe()) {
            Robot.drive.rawDrive(0.0, 0.0);
        }
        return isSafe();
    }
}