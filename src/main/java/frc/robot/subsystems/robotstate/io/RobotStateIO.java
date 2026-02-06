package frc.robot.subsystems.robotstate.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Defines the contract for robot state telemetry inputs that AdvantageKit will log.
 */
public interface RobotStateIO {

    /**
     * Container of robot state telemetry fields that AdvantageKit will persist.
     */
    @AutoLog
    public static class RobotStateIOInputs {
        /**
         * Latest odometry pose in meters and radians.
         */
        public Pose2d  odometryPose                   = new Pose2d();

        /**
         * Latest fused pose estimate in meters and radians.
         */
        public Pose2d  estimatedPose                  = new Pose2d();

        /**
         * Most recent vision pose in meters and radians.
         */
        public Pose2d  lastVisionPose                 = new Pose2d();

        /**
         * Timestamp of the last vision pose in seconds.
         */
        public double  lastVisionTimestampSeconds     = Double.NaN;

        /**
         * True when a vision measurement has been received.
         */
        public boolean hasVisionMeasurement           = false;

        /**
         * True when vision measurements are allowed to blend into the estimate.
         */
        public boolean enableVisionFusion             = true;

        /**
         * Blend factor used to interpolate between odometry and vision poses.
         */
        public double  visionBlendFactor              = 0.5;

        /**
         * Maximum age of a vision measurement in seconds before it is rejected.
         */
        public double  visionMeasurementMaxAgeSeconds = 0.0;
    }

    /**
     * Refreshes the inputs structure with the latest robot state telemetry.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(RobotStateIOInputs inputs);
}
