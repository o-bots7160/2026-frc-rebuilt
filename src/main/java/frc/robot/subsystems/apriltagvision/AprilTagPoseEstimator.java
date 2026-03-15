package frc.robot.subsystems.apriltagvision;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.apriltagvision.io.AprilTagVisionIO.PoseObservation;

/**
 * Processes AprilTag pose observations and produces vision measurements suitable for pose estimation.
 * <p>
 * This class encapsulates the filtering and standard deviation calculation logic, making it testable without requiring PhotonVision, subsystem
 * infrastructure, or AdvantageKit logging.
 * </p>
 */
public class AprilTagPoseEstimator {

    /**
     * Parameters for pose estimation filtering and uncertainty calculation.
     *
     * @param fieldLengthMeters     field length for bounds checking in meters
     * @param fieldWidthMeters      field width for bounds checking in meters
     * @param maxAmbiguity          maximum dimensionless ambiguity for single-tag observations
     * @param linearStdDevBaseline  baseline standard deviation for x/y in meters at 1 meter with 1 tag
     * @param angularStdDevBaseline baseline standard deviation for rotation in radians at 1 meter with 1 tag
     * @param maxResidualTranslationMeters maximum allowed translation disagreement with the reference pose
     * @param maxResidualRotationRadians maximum allowed rotation disagreement with the reference pose for multi-tag solves
     */
    public record Params(
            double fieldLengthMeters,
            double fieldWidthMeters,
            double maxAmbiguity,
            double linearStdDevBaseline,
            double angularStdDevBaseline,
            double maxResidualTranslationMeters,
            double maxResidualRotationRadians) {
    }

    /**
     * A validated vision measurement ready for pose estimation fusion.
     *
     * @param pose               the estimated robot pose in meters and radians
     * @param timestampSeconds   the timestamp when the observation was captured in seconds
     * @param standardDeviations uncertainty in x, y (meters) and rotation (radians)
     */
    public record VisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            Matrix<N3, N1> standardDeviations) {
    }

    /** Huge standard deviation for single-tag rotation observations, effectively disabling rotation updates. */
    private static final double SINGLE_TAG_ROTATION_STD_DEV_RADIANS = 1.0e6;

    private Params               params;

    /** True after the first measurement is accepted, enabling residual gating for all subsequent observations. */
    private boolean              referencePoseInitialized = false;

    /**
     * Creates a new AprilTagPoseEstimator.
     *
     * @param params configuration parameters for filtering and uncertainty
     */
    public AprilTagPoseEstimator(Params params) {
        this.params = params;
    }

    /**
     * Returns the active estimator parameters.
     *
     * @return current filtering and uncertainty parameters
     */
    public Params getParams() {
        return params;
    }

    /**
     * Replaces the active estimator parameters so live-tuned values take effect without rebuilding the subsystem.
     *
     * @param params updated filtering and uncertainty parameters
     */
    public void setParams(Params params) {
        this.params = params;
    }

    /**
     * Returns whether a reference pose has been established for residual gating.
     *
     * @return true after the first measurement has been accepted
     */
    public boolean isReferencePoseInitialized() {
        return referencePoseInitialized;
    }

    /**
     * Explicitly marks the reference pose as initialized or uninitialized.
     * <p>
     * Call this after a pose reset so residual gating resumes immediately, or clear it to allow the next observation through without residual checks.
     * </p>
     *
     * @param initialized true to enable residual gating, false to disable until the next accepted measurement
     */
    public void setReferencePoseInitialized(boolean initialized) {
        this.referencePoseInitialized = initialized;
    }

    /**
     * Processes a pose observation and returns a measurement if accepted.
     * <p>
     * Standard deviations are scaled by distance squared divided by tag count, using meters for distance.
     * </p>
     *
     * @param observation   the raw pose observation from vision
     * @param referencePose current odometry/reference pose used for residual gating
     * @return the validated measurement, or empty if the observation was rejected
     */
    public Optional<VisionMeasurement> estimate(PoseObservation observation, Pose2d referencePose) {
        // Stop early if the camera data fails our safety checks.
        if (shouldReject(observation, referencePose)) {
            return Optional.empty();
        }

        // Farther tags and fewer tags mean less confidence, so scale up the uncertainty.
        double factor        = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev  = params.linearStdDevBaseline() * factor;
        double angularStdDev = observation.tagCount() > 1
                ? params.angularStdDevBaseline() * factor
                : SINGLE_TAG_ROTATION_STD_DEV_RADIANS;

        // Package the pose, timestamp, and uncertainty so the estimator can fuse it.
        referencePoseInitialized = true;
        return Optional.of(new VisionMeasurement(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
    }

    /**
     * Determines whether a pose observation should be rejected.
     * <p>
     * An observation is rejected when no tags were detected, when a single-tag observation exceeds the dimensionless ambiguity threshold, or when the
     * estimated pose falls outside the field boundaries in meters. If a valid odometry reference pose is available, the observation is also rejected
     * when its translation disagrees with the reference by more than {@code maxResidualTranslationMeters}, or when a multi-tag solve disagrees in
     * rotation by more than {@code maxResidualRotationRadians}.
     * </p>
     *
     * @param observation   the pose observation to evaluate
     * @param referencePose current odometry/reference pose used for residual gating
     * @return true if the observation should be rejected
     */
    public boolean shouldReject(PoseObservation observation, Pose2d referencePose) {
        // If we see no tags, there is no useful pose to trust.
        if (observation.tagCount() == 0) {
            return true;
        }

        // Single-tag measurements can be ambiguous; reject if too uncertain.
        if (observation.tagCount() == 1
                && observation.ambiguity() > params.maxAmbiguity()) {
            return true;
        }

        // Reject poses that fall outside the field rectangle.
        var pose = observation.pose();
        if (pose.getX() < 0.0 || pose.getX() > params.fieldLengthMeters()) {
            return true;
        }
        if (pose.getY() < 0.0 || pose.getY() > params.fieldWidthMeters()) {
            return true;
        }

        Pose2d pose2d = pose.toPose2d();
        if (referencePoseInitialized
                && pose2d.getTranslation().getDistance(referencePose.getTranslation()) > params.maxResidualTranslationMeters()) {
            return true;
        }

        if (observation.tagCount() > 1
                && referencePoseInitialized
                && Math.abs(MathUtil.angleModulus(
                        pose2d.getRotation().minus(referencePose.getRotation()).getRadians())) > params.maxResidualRotationRadians()) {
            return true;
        }

        return false;
    }

}
