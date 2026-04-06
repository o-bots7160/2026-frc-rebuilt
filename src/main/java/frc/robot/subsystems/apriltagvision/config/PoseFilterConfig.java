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
     * Number of vision poses to accept without checking odometry deviation at startup.
     * <p>
     * At startup the robot's odometry initializes at (0, 0). Because the real robot is placed elsewhere on the field, every vision observation will
     * exceed {@link #maximumPoseDeviationMeters} and be rejected. Setting this to a value greater than zero lets that many observations through
     * regardless of deviation so the pose estimator can lock onto the correct field position. After the configured count of measurements have been
     * accepted, normal deviation filtering resumes. Set to 0 to disable this behavior.
     * </p>
     */
    public int initialPoseAcceptanceCount = 3;

    /**
     * Standard deviation for linear (x/y) pose measurements at 1 meter with a single tag. Used as the baseline that is scaled by tag distance and
     * count to produce the final measurement uncertainty.
     */
    public double linearStandardDeviationBaseline = 0.08;

    /**
     * Standard deviation for angular (rotation) pose measurements at 1 meter with a single tag. Used as the baseline that is scaled by tag distance
     * and count to produce the final measurement uncertainty.
     */
    public double angularStandardDeviationBaseline = 1.0;

    /**
     * Maximum pose ambiguity allowed for single-tag observations. Observations with higher ambiguity are rejected. Complements
     * {@link #maximumMultiTagAmbiguity} which applies to multi-tag observations.
     */
    public double maximumAmbiguity = 0.15;

    /**
     * Duration in seconds that {@code HasVisibleTags} stays true after the last tag is seen.
     * <p>
     * Camera frames arrive at the camera's own FPS, which may not align with the 50 Hz robot loop. On cycles where no new frames are available,
     * the tag-ID set is empty and the visibility boolean would flicker. This hold-off keeps the boolean stable for the configured duration after
     * tags were last observed, preventing misleading flicker on the dashboard.
     * </p>
     */
    public double tagVisibilityHoldOffSeconds = 0.25;

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

    /**
     * Returns the number of initial pose observations to accept without odometry deviation checking.
     *
     * @return number of poses to accept at startup before enforcing deviation limits, or 0 to disable
     */
    public int getInitialPoseAcceptanceCount() {
        return initialPoseAcceptanceCount;
    }

    /**
     * Returns the linear standard deviation baseline for pose estimation.
     *
     * @return linear std dev baseline in meters
     */
    public double getLinearStandardDeviationBaseline() {
        return readTunableNumber("linearStandardDeviationBaseline", linearStandardDeviationBaseline);
    }

    /**
     * Returns the angular standard deviation baseline for pose estimation.
     *
     * @return angular std dev baseline in radians
     */
    public double getAngularStandardDeviationBaseline() {
        return readTunableNumber("angularStandardDeviationBaseline", angularStandardDeviationBaseline);
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
     * Returns the hold-off duration that keeps {@code HasVisibleTags} true after tags disappear.
     *
     * @return hold-off duration in seconds
     */
    public double getTagVisibilityHoldOffSeconds() {
        return readTunableNumber("tagVisibilityHoldOffSeconds", tagVisibilityHoldOffSeconds);
    }
}
