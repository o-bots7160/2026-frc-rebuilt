package frc.robot.subsystems.apriltagvision.config;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.shared.config.AbstractSubsystemConfig;

/**
 * Configuration bundle for the AprilTag vision subsystem. Values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 */
public class AprilTagVisionSubsystemConfig extends AbstractSubsystemConfig {

    /**
     * Nested class for JSON deserialization of camera transforms.
     * <p>
     * Translation values are in meters and rotations are in radians.
     * </p>
     */
    public static class CameraTransform {
        /**
         * Robot-to-camera X offset in meters (forward positive).
         */
        public double x;

        /**
         * Robot-to-camera Y offset in meters (left positive).
         */
        public double y;

        /**
         * Robot-to-camera Z offset in meters (up positive).
         */
        public double z;

        /**
         * Robot-to-camera roll in radians.
         */
        public double roll;

        /**
         * Robot-to-camera pitch in radians.
         */
        public double pitch;

        /**
         * Robot-to-camera yaw in radians.
         */
        public double yaw;

        /**
         * Converts this config object to a WPILib Transform3d.
         *
         * @return Transform3d representing the robot-to-camera transform
         */
        public Transform3d toTransform3d() {
            return new Transform3d(
                    new Translation3d(x, y, z),
                    new Rotation3d(roll, pitch, yaw));
        }
    }

    /**
     * Camera names and their robot-to-camera transforms. Each transform defines the camera position relative to robot center: (x forward, y left, z
     * up).
     */
    public Map<String, CameraTransform> cameras;

    /**
     * Standard deviation for angular (rotation) pose measurements at 1 meter with a single tag.
     */
    public double                       angularStandardDeviationBaseline;

    /**
     * Standard deviation for linear (x/y) pose measurements at 1 meter with a single tag.
     */
    public double                       linearStandardDeviationBaseline;

    /**
     * Maximum pose ambiguity allowed for single-tag observations. Observations with higher ambiguity are rejected.
     */
    public double                       maximumAmbiguity;

    /**
     * Maximum allowed translation delta between odometry and a vision pose before the observation is rejected.
     */
    public double                       maximumResidualTranslationMeters = 1.0;

    /**
     * Maximum allowed heading delta between odometry and a multi-tag vision pose before the observation is rejected.
     */
    public double                       maximumResidualRotationDegrees   = 30.0;

    /**
     * Returns the angular standard deviation baseline for pose estimation.
     *
     * @return angular std dev baseline (radians)
     */
    public double getAngularStandardDeviationBaseline() {
        return readTunableNumber("angularStandardDeviationBaseline", angularStandardDeviationBaseline);
    }

    /**
     * Returns the linear standard deviation baseline for pose estimation.
     *
     * @return linear std dev baseline (meters)
     */
    public double getLinearStandardDeviationBaseline() {
        return readTunableNumber("linearStandardDeviationBaseline", linearStandardDeviationBaseline);
    }

    /**
     * Returns the maximum ambiguity threshold for single-tag observations.
     *
     * @return max ambiguity (dimensionless)
     */
    public double getMaximumAmbiguity() {
        return readTunableNumber("maximumAmbiguity", maximumAmbiguity);
    }

    /**
     * Returns the maximum allowed translation residual for a vision observation.
     *
     * @return translation residual threshold in meters
     */
    public double getMaximumResidualTranslationMeters() {
        return readTunableNumber("maximumResidualTranslationMeters", maximumResidualTranslationMeters);
    }

    /**
     * Returns the maximum allowed rotation residual for a multi-tag vision observation.
     *
     * @return rotation residual threshold in radians
     */
    public double getMaximumResidualRotationRadians() {
        return readTunableDegreesAsRadians("maximumResidualRotationDegrees", maximumResidualRotationDegrees);
    }

    /**
     * Returns the configured camera map.
     *
     * @return map of camera names to robot-to-camera transforms
     */
    public Map<String, CameraTransform> getCameras() {
        return cameras;
    }
}
