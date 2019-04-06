package frc.robot.util;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public abstract class ControlLoop implements Runnable {
    private static final double kDefaultPeriod = 0.02;

    private double kPeriod;
    private Notifier notifier_;
    private double timestamp_ = 0.0;
    private double dt_ = 0.0;
    private boolean running_ = false;
    private boolean shouldLoop_ = false;

    /**
     * Control loop constructor. Uses default period.
     */
    public ControlLoop() {
        this(kDefaultPeriod);
    }

    /**
     * Control loop constructor.
     * @param period period in seconds
     */
    public ControlLoop(double period) {
        kPeriod = period;
        notifier_ = new Notifier(this::runLoop);
    }

    @Override
    public final void run() {
        running_ = true;
        if(init()) {
            notifier_.startPeriodic(kPeriod);
        }
    }

    public void startPeriodic() {
        shouldLoop_ = true;
    }

    private final void runLoop() {
        double now = Timer.getFPGATimestamp();
        dt_ = now - timestamp_;
        timestamp_ = now;
        if(!running_ || Thread.interrupted()) {
            notifier_.stop();
            end();
            return;
        }
        if(shouldLoop_) {
            loop();
        }
    }

    public synchronized final void stopLoop() {
        running_ = false;
        notifier_.stop();
        end();
    }

    public final double getDt() {
        return dt_;
    }

    public final double getTimestamp() {
        return timestamp_;
    }

    /**
     * The initialization method for the control loop. Return true if the thread should continue after init to run the loop.
     */
    public abstract boolean init();

    public abstract void loop();
    public abstract void end();
}