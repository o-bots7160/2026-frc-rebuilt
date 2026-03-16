package frc.robot.subsystems.apriltagvision.config;

import frc.robot.shared.config.AbstractConfig;

/**
 * Filtering and trust parameters for AprilTag pose estimation.
 * <p>
 * Group all pose-quality filters in one place so they can be tuned together from the dashboard or adjusted in the deploy JSON without touching the
 * main vision config. Every numeric field is live-tunable via SmartDashboard when the robot is not connected to FMS.
 * </p>
 */
public class PoseFilterConfig extends AbstractConfig {

    /**
     * Maximum average tag distance in meters. Observations where all detected tags are farther than this threshold are rejected outright because
     * accuracy degrades quadratically with distance.
     */
    public double maximumTagDistanceMeters   = 5.0;

    /**
     * Maximum allowed deviation in meters between the vision-estimated pose and the current odometry pose. Observations that disagree with odometry
     * by more than this distance are rejected to prevent single-frame outliers from jerking the fused pose.
     */
    public double maximumPoseDeviationMeters = 1.5;

    /**
     * Maximum ambiguity for multi-tag PnP solves. Degenerate multi-tag configurations (e.g., two coplanar tags at extreme viewing angles) can produce
     * ambiguity above this threshold and should be rejected.
     */
    public double maximumMultiTagAmbiguity   = 0.2;

    /**
     * Maximum absolute Z height in meters for the estimated robot pose. A ground robot should never report itself as significantly above or below the
     * floor. Observations exceeding this threshold are rejected as physically impossible.
     */
    public double maximumZHeightMeters       = 0.5;

    /**
     * AprilTag IDs to ignore. Observations that contain only ignored tags are rejected. Useful for blacklisting known-problematic tags (damaged
     * prints, field defects, or tags that cause persistent jitter in a paired configuration) without requiring a code change.
     */
    public int[]  ignoredTagIds              = new int[0];

    /**
     * Standard deviation multiplier applied when the set of observed tag IDs changes between consecutive accepted observations from the same camera.
     * <p>
     * On the 2026 Rebuilt field many structures mount two tags on the same face (~35 cm apart). When a camera alternates between detecting one tag
     * and the other, the computed robot pose oscillates. By multiplying the standard deviations on "tag-switch" frames, the Kalman filter trusts
     * these observations less and smooths the jitter without rejecting data.
     * </p>
     */
    public double tagSwitchStdDevMultiplier  = 3.0;

    /**
     * Returns the maximum average tag distance threshold.
     *
     * @return maximum tag distance in meters
     */
    public double getMaximumTagDistanceMeters() {
        return readTunableNumber("maximumTagDistanceMeters", maximumTagDistanceMeters);
    }

    /**
     * Returns the maximum pose deviation threshold.
     *
     * @return maximum deviation between vision and odometry in meters
     */
    public double getMaximumPoseDeviationMeters() {
        return readTunableNumber("maximumPoseDeviationMeters", maximumPoseDeviationMeters);
    }

    /**
     * Returns the maximum ambiguity threshold for multi-tag observations.
     *
     * @return maximum multi-tag ambiguity (dimensionless)
     */
    public double getMaximumMultiTagAmbiguity() {
        return readTunableNumber("maximumMultiTagAmbiguity", maximumMultiTagAmbiguity);
    }

    /**
     * Returns the maximum absolute Z height for pose validation.
     *
     * @return maximum Z height in meters
     */
    public double getMaximumZHeightMeters() {
        return readTunableNumber("maximumZHeightMeters", maximumZHeightMeters);
    }

    /**
     * Returns the tag IDs to ignore during pose estimation.
     *
     * @return array of ignored tag IDs
     */
    public int[] getIgnoredTagIds() {
        return ignoredTagIds;
    }

    /**
     * Returns the standard deviation multiplier for tag-switch frames.
     *
     * @return multiplier applied when observed tag IDs change between consecutive frames
     */
    public double getTagSwitchStdDevMultiplier() {
        return readTunableNumber("tagSwitchStdDevMultiplier", tagSwitchStdDevMultiplier);
    }
}
