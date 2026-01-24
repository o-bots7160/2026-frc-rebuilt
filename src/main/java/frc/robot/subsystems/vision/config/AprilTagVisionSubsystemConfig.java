package frc.robot.subsystems.vision.config;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration bundle for the AprilTag vision subsystem. Values are mirrored
 * to SmartDashboard so they can be tuned live without redeploying firmware.
 */
public class AprilTagVisionSubsystemConfig extends AbstractConfig {

    /**
     * Nested class for JSON deserialization of camera transforms.
     */
    public static class CameraTransform {
        public double x;

        public double y;

        public double z;

        public double roll;

        public double pitch;

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
     * Cameras name and their robot-to-camera transform. Each transform defines the camear position relative to robot center: (x forward, y left, z
     * up).
     */
    public Map<String, CameraTransform> cameras;

    /**
     * Standard deviation for angular (rotation) pose measurements at 1 meter with a single tag.
     */
    public double angularStandardDeviationBaseline;

    /**
     * Standard deviation for linear (x/y) pose measurements at 1 meter with a single tag.
     */
    public double linearStandardDeviationBaseline;

    /**
     * Maximum pose ambiguity allowed for single-tag observations. Observations with higher ambiguity are rejected.
     */
    public double maximumAmbiguity;

    /**
     * Supplies the angular standard deviation baseline for pose estimation.
     *
     * @return supplier yielding the current angular std dev baseline (radians)
     */
    public Supplier<Double> getAngularStandardDeviationBaseline() {
        return () -> readTunableNumber("angularStandardDeviationBaseline", angularStandardDeviationBaseline);
    }

    /**
     * Supplies the linear standard deviation baseline for pose estimation.
     *
     * @return supplier yielding the current linear std dev baseline (meters)
     */
    public Supplier<Double> getLinearStandardDeviationBaseline() {
        return () -> readTunableNumber("linearStandardDeviationBaseline", linearStandardDeviationBaseline);
    }

    /**
     * Supplies the maximum ambiguity threshold for single-tag observations.
     *
     * @return supplier yielding the current max ambiguity value
     */
    public Supplier<Double> getMaximumAmbiguity() {
        return () -> readTunableNumber("maximumAmbiguity", maximumAmbiguity);
    }

    /**
     * Returns the configured camera names.
     *
     * @return array of camera names matching PhotonVision configuration
     */
    public Map<String, CameraTransform> getCameras() {
        return cameras;
    }
}
