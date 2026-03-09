package frc.robot.subsystems.robotpose.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Defines the contract for robot pose telemetry inputs that AdvantageKit will log.
 */
public interface RobotPoseIO {

    /**
     * Container of robot pose telemetry fields that AdvantageKit will persist.
     */
    @AutoLog
    public static class RobotPoseIOInputs {
        /**
         * Latest fused pose estimate in meters and radians.
         */
        public Pose2d  estimatedPose              = new Pose2d();

        /**
         * Raw odometry-only pose from wheel encoders and gyro, without vision corrections.
         * <p>
         * Useful as an AdvantageScope ghost bot to visualize odometry drift compared to the fused estimate.
         * </p>
         */
        public Pose2d  odometryOnlyPose           = new Pose2d();

        /**
         * Most recent vision pose in meters and radians.
         */
        public Pose2d  lastVisionPose             = new Pose2d();

        /**
         * Timestamp of the last vision pose in seconds.
         */
        public double  lastVisionTimestampSeconds = Double.NaN;

        /**
         * True when a vision measurement has been received.
         */
        public boolean hasVisionMeasurement       = false;

        /**
         * True when vision measurements are allowed to blend into the estimate.
         */
        public boolean enableVisionFusion         = true;
    }

    /**
     * Refreshes the inputs structure with the latest robot pose telemetry.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(RobotPoseIOInputs inputs);
}
