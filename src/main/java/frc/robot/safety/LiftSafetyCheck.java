package frc.robot.safety;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Robot;

/**
 * Lift safety check. Ensures that the lift...
 *  1) Doesn't move past it's lowest state (which will WRECK the chain lol)
 */
public class LiftSafetyCheck {
    /**
     * The safety condition.
     * @return boolean true if the robot is performing safe action, false otherwise
     */
    public static boolean isSafe() {
        return !(Robot.lift.bottomHal.get() && Robot.lift.master.getMotorOutputPercent() < 0.0); // unsafe if bottom hall true and lift tries to go down
    }

    public static boolean ensureSafety() {
        if(!isSafe()) {
            Robot.lift.master.set(ControlMode.PercentOutput, 0.0);
        }
        return isSafe();
    }
}