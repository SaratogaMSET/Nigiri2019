package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.FishyMath;
import frc.robot.util.Kinematics;
import frc.robot.util.RigidTransform2d;
import frc.robot.util.Rotation2d;
import frc.robot.util.Translation2d;
import frc.robot.util.Twist2d;

public class RobotPose {
    private static RobotPose instance_ = new RobotPose();
    public synchronized static RobotPose getInstance() {
        return instance_;
    }

    public static class RobotPoseEstimator implements Runnable {
        public static final double kPeriod = 0.005;
        private final Notifier notifier_;
        private double timestamp_ = 0;
        private double dt_ = 0;
        private boolean running_ = false;

        private RobotPose pose;

        private double left_encoder_prev_distance_ = 0;
        private double right_encoder_prev_distance_ = 0;

        public RobotPoseEstimator() {
            notifier_ = new Notifier(this::loop);
        }

        @Override
        public void run() {
            pose = RobotPose.getInstance();
            if(Robot.drive != null) {
                synchronized(Robot.drive) {
                    left_encoder_prev_distance_ = Robot.drive.getLeftEncoderDistance();
                    right_encoder_prev_distance_ = Robot.drive.getRightEncoderDistance();
                }
            }
            running_ = true;
            notifier_.startPeriodic(kPeriod);
        }

        public void stop() {
            notifier_.stop();
        }

        public void loop() {
            double now = Timer.getFPGATimestamp();
            dt_ = now - timestamp_;
            timestamp_ = now;
            if(Robot.drive == null) return;
            if(!running_ || Thread.interrupted()) {
                notifier_.stop();
                return;
            }
            synchronized(Robot.drive) {
                final double left_distance = Robot.drive.getLeftEncoderDistance();
                final double right_distance = Robot.drive.getRightEncoderDistance();
                final Rotation2d gyro_angle = Rotation2d.fromDegrees(FishyMath.boundThetaNeg180to180(-Robot.gyro.getGyroAngle()));
                final Twist2d odometry_velocity = generateOdometryFromSensors(left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
                final RigidTransform2d finalPose = Kinematics.integrateForwardKinematics(RobotPose.getPose(), odometry_velocity);
                RobotPose.setPose(finalPose);
                left_encoder_prev_distance_ = left_distance;
                right_encoder_prev_distance_ = right_distance;
            }
        }

        public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
            final RigidTransform2d last_measurement = RobotPose.getPose();
            final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle);
            return delta;
        }

        public double getDt() {
            return dt_;
        }
    }

    private static RobotPoseEstimator runnable_ = new RobotPoseEstimator();
    public synchronized static RobotPoseEstimator getRunnable() {
        return runnable_;
    }


    private double ROBOT_X = 0.0;
    private double ROBOT_Y = 0.0;
    private double ROBOT_HEADING = 0.0;

    public static synchronized double getX() {
        return getInstance().ROBOT_X;
    }
    public static synchronized double getY() {
        return getInstance().ROBOT_Y;
    }
    public static synchronized double getHeading() {
        return getInstance().ROBOT_HEADING;
    }
    public static synchronized RigidTransform2d getPose() {
        RobotPose pose = getInstance();
        return new RigidTransform2d(new Translation2d(pose.ROBOT_X, pose.ROBOT_Y), Rotation2d.fromRadians(pose.ROBOT_HEADING));
    }
    
    public static synchronized void setPose(double x, double y, double heading) {
        RobotPose pose = getInstance();
        synchronized(pose) {
            pose.ROBOT_X = x;
            pose.ROBOT_Y = y;
            pose.ROBOT_HEADING = heading;
        } 
    }

    public static synchronized void setPose(RigidTransform2d pose) {
        RobotPose robotPose = getInstance();
        synchronized(robotPose) {
            robotPose.ROBOT_X = pose.getTranslation().x();
            robotPose.ROBOT_Y = pose.getTranslation().y();
            robotPose.ROBOT_HEADING = pose.getRotation().getRadians();
        } 
    }
}