package frc.robot.subsystems.apriltagvision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

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
     * Reason a pose observation was rejected by the estimator. Used for per-reason telemetry so operators can see which filter is triggering during a
     * match.
     */
    public enum RejectionReason {
        /** No tags were detected in the observation. */
        NO_TAGS,
        /** Single-tag observation exceeded the ambiguity threshold. */
        SINGLE_TAG_AMBIGUITY,
        /** Multi-tag observation exceeded the ambiguity threshold. */
        MULTI_TAG_AMBIGUITY,
        /** Estimated pose is outside the field boundaries. */
        OUTSIDE_FIELD,
        /** Average tag distance exceeded the maximum allowed distance. */
        TOO_FAR,
        /** Estimated Z height is physically unreasonable. */
        BAD_Z_HEIGHT,
        /** Vision pose deviates too far from the current odometry pose. */
        ODOMETRY_DEVIATION,
        /** All observed tags are in the ignored tag list. */
        IGNORED_TAGS
    }

    /**
     * Result of evaluating a pose observation. Contains either an accepted measurement or a rejection reason for telemetry.
     *
     * @param measurement     the validated vision measurement, or empty if rejected
     * @param rejectionReason the reason for rejection, or empty if accepted
     */
    public record EstimationResult(
            Optional<VisionMeasurement> measurement,
            Optional<RejectionReason> rejectionReason) {

        /**
         * Creates an accepted result containing a vision measurement.
         *
         * @param measurement the validated measurement to forward to the pose estimator
         * @return accepted result
         */
        public static EstimationResult accepted(VisionMeasurement measurement) {
            return new EstimationResult(Optional.of(measurement), Optional.empty());
        }

        /**
         * Creates a rejected result with the given reason.
         *
         * @param reason why the observation was rejected
         * @return rejected result
         */
        public static EstimationResult rejected(RejectionReason reason) {
            return new EstimationResult(Optional.empty(), Optional.of(reason));
        }
    }

    /**
     * Parameters for pose estimation filtering and uncertainty calculation.
     *
     * @param fieldLengthMeters            field length for bounds checking in meters
     * @param fieldWidthMeters             field width for bounds checking in meters
     * @param maxAmbiguity                 maximum dimensionless ambiguity for single-tag observations
     * @param linearStdDevBaseline         baseline standard deviation for x/y in meters at 1 meter with 1 tag
     * @param angularStdDevBaseline        baseline standard deviation for rotation in radians at 1 meter with 1 tag
     * @param maxTagDistanceMeters         maximum average tag distance in meters before rejection
     * @param maxPoseDeviationMeters       maximum deviation from odometry in meters before rejection
     * @param maxMultiTagAmbiguity         maximum ambiguity for multi-tag observations
     * @param maxZHeightMeters             maximum absolute Z height in meters for the estimated pose
     * @param ignoredTagIds                tag IDs to ignore during estimation
     * @param tagSwitchStdDevMultiplier    standard deviation multiplier when tag IDs change between frames
     * @param initialPoseAcceptanceCount   number of poses to accept without odometry deviation checking at startup, or 0 to disable
     * @param recoveryMinimumTagCount       minimum number of tags required to recover from odometry drift
     * @param recoveryRequiredConsecutiveFrames consecutive consistent frames required before recovery
     * @param recoveryMaxTranslationMeters maximum translation change between recovery frames in meters
     * @param recoveryMaxRotationRadians   maximum heading change between recovery frames in radians
     */
    public record Params(
            double fieldLengthMeters,
            double fieldWidthMeters,
            double maxAmbiguity,
            double linearStdDevBaseline,
            double angularStdDevBaseline,
            double maxTagDistanceMeters,
            double maxPoseDeviationMeters,
            double maxMultiTagAmbiguity,
            double maxZHeightMeters,
            int[] ignoredTagIds,
            double tagSwitchStdDevMultiplier,
            int initialPoseAcceptanceCount,
            int recoveryMinimumTagCount,
            int recoveryRequiredConsecutiveFrames,
            double recoveryMaxTranslationMeters,
            double recoveryMaxRotationRadians) {
    }

    /**
     * Consecutive vision poses being evaluated as a possible odometry-drift recovery.
     *
     * @param pose  most recent candidate pose in meters and radians
     * @param count number of mutually consistent consecutive poses
     */
    private record RecoveryCandidate(Pose2d pose, int count) {
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

    /**
     * Huge standard deviation assigned to single-tag rotation observations.
     * <p>
     * Single-tag PnP solves produce very noisy rotation estimates. By assigning an enormous angular uncertainty the Kalman filter effectively ignores
     * the rotation component while still fusing the more reliable x/y translation.
     * </p>
     */
    private static final double             SINGLE_TAG_ROTATION_STD_DEV_RADIANS = 1.0e6;

    private final Params                    params;

    private final Supplier<Pose2d>          odometryPoseSupplier;

    private final Set<Integer>              ignoredTagIdSet;

    private final Map<String, Set<Integer>> lastTagIdsByCamera                  = new HashMap<>();

    private final Map<String, RecoveryCandidate> recoveryCandidatesByCamera     = new HashMap<>();

    /** Counts accepted measurements so the initial pose acceptance window can expire. */
    private int                             acceptedMeasurementCount;

    /**
     * When true, the odometry deviation filter is bypassed so every geometrically valid observation is accepted.
     * <p>
     * Activated during the disabled period on real hardware so the pose estimator continuously tracks vision while operators
     * position the robot on the field. All other safety filters (ambiguity, field bounds, Z height, tag distance) remain active.
     * </p>
     */
    private boolean                         poseCalibrationActive;

    /** Translation difference between the latest vision pose and current fused pose, in meters. */
    private double                          lastOdometryDeviationMeters         = Double.NaN;

    /** Number of consistent frames in the latest camera's recovery candidate sequence. */
    private int                             lastRecoveryCandidateCount;

    /** True when the latest accepted measurement bypassed the odometry deviation filter through recovery. */
    private boolean                         lastMeasurementUsedRecovery;

    /**
     * Creates a new AprilTagPoseEstimator.
     *
     * @param params               configuration parameters for filtering and uncertainty
     * @param odometryPoseSupplier supplier for the current odometry pose used for consistency checking
     */
    public AprilTagPoseEstimator(Params params, Supplier<Pose2d> odometryPoseSupplier) {
        this.params               = params;
        this.odometryPoseSupplier = odometryPoseSupplier;
        this.ignoredTagIdSet      = Arrays.stream(params.ignoredTagIds())
                .boxed()
                .collect(Collectors.toSet());
    }

    /**
     * Returns whether the estimator is still within the initial pose acceptance window.
     * <p>
     * While this returns true, the odometry deviation filter is bypassed so the pose estimator can lock onto a real
     * field position at startup.
     * </p>
     *
     * @return true if the accepted measurement count has not yet reached the configured initial acceptance count
     */
    public boolean isWithinInitialAcceptanceWindow() {
        return params.initialPoseAcceptanceCount() > 0
                && acceptedMeasurementCount < params.initialPoseAcceptanceCount();
    }

    /**
     * Returns whether pose calibration mode is currently active.
     *
     * @return true when the odometry deviation filter is bypassed for continuous calibration
     */
    public boolean isPoseCalibrationActive() {
        return poseCalibrationActive;
    }

    /**
     * Enables or disables pose calibration mode.
     * <p>
     * When enabled, the odometry deviation filter is bypassed so every geometrically valid vision observation is accepted. All other safety filters
     * (ambiguity, field bounds, Z height, tag distance) remain active. Enable this while the robot is disabled so operators can continuously
     * calibrate the pose by repositioning the robot on the field.
     * </p>
     *
     * @param active true to bypass the odometry deviation filter, false to restore normal filtering
     */
    public void setPoseCalibrationActive(boolean active) {
        this.poseCalibrationActive = active;
    }

    /**
     * Returns the latest measured translation difference between vision and the fused pose.
     *
     * @return latest vision-to-estimator translation difference in meters, or {@link Double#NaN} when unavailable
     */
    public double getLastOdometryDeviationMeters() {
        return lastOdometryDeviationMeters;
    }

    /**
     * Returns the current recovery sequence length for the most recently evaluated camera.
     *
     * @return number of consecutive, mutually consistent recovery candidate frames
     */
    public int getLastRecoveryCandidateCount() {
        return lastRecoveryCandidateCount;
    }

    /**
     * Returns whether the latest accepted measurement used the odometry-drift recovery path.
     *
     * @return true when recovery bypassed the normal odometry deviation limit
     */
    public boolean didLastMeasurementUseRecovery() {
        return lastMeasurementUsedRecovery;
    }

    /**
     * Processes a pose observation for initial pose calibration, bypassing the odometry deviation filter.
     * <p>
     * Use this method during the disabled period to accept all geometrically valid observations regardless of how far they deviate from the current
     * odometry estimate. This lets operators reposition the robot on the field and see the pose converge in real time. All other safety filters
     * (ambiguity, field bounds, Z height, tag distance) remain active so physically impossible poses are still rejected.
     * </p>
     *
     * @param observation the raw pose observation from vision
     * @param cameraName  the name of the camera that produced this observation
     * @return estimation result containing the measurement or rejection reason
     */
    public EstimationResult estimateForCalibration(PoseObservation observation, String cameraName) {
        lastOdometryDeviationMeters = Double.NaN;
        lastRecoveryCandidateCount  = 0;
        lastMeasurementUsedRecovery = false;
        recoveryCandidatesByCamera.remove(cameraName);

        Optional<RejectionReason> rejection = checkRejection(observation, true, cameraName);
        if (rejection.isPresent()) {
            return EstimationResult.rejected(rejection.get());
        }

        return buildAcceptedResult(observation, cameraName);
    }

    /**
     * Processes a pose observation and returns an estimation result.
     * <p>
     * Standard deviations are scaled by distance squared divided by tag count, using meters for distance. When the set of observed tag IDs changes
     * between consecutive frames from the same camera, the standard deviations are further multiplied by the tag-switch multiplier to dampen jitter
     * from same-face tag pairs.
     * </p>
     *
     * @param observation the raw pose observation from vision
     * @param cameraName  the name of the camera that produced this observation
     * @return estimation result containing the measurement or rejection reason
     */
    public EstimationResult estimate(PoseObservation observation, String cameraName) {
        lastOdometryDeviationMeters = Double.NaN;
        lastMeasurementUsedRecovery = false;
        lastRecoveryCandidateCount  = 0;

        Optional<RejectionReason> rejection = checkRejection(observation, false, cameraName);
        if (rejection.isPresent()) {
            if (rejection.get() != RejectionReason.ODOMETRY_DEVIATION) {
                recoveryCandidatesByCamera.remove(cameraName);
            }
            return EstimationResult.rejected(rejection.get());
        }

        acceptedMeasurementCount++;
        return buildAcceptedResult(observation, cameraName);
    }

    /**
     * Builds an accepted estimation result from a pose observation that has already passed rejection filters.
     * <p>
     * Computes distance-scaled standard deviations and applies tag-switch dampening so the Kalman filter weights the measurement appropriately.
     * </p>
     *
     * @param observation the pose observation that passed filtering
     * @param cameraName  the camera that produced the observation, used for tag-switch tracking
     * @return accepted estimation result with computed standard deviations
     */
    private EstimationResult buildAcceptedResult(PoseObservation observation, String cameraName) {
        // Farther tags and fewer tags mean less confidence, so scale up the uncertainty.
        double factor              = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev        = params.linearStdDevBaseline() * factor;
        double angularStdDev       = observation.tagCount() > 1
                ? params.angularStdDevBaseline() * factor
                : SINGLE_TAG_ROTATION_STD_DEV_RADIANS;

        // Apply tag-switch dampening when the camera switches between different tags.
        double tagSwitchMultiplier = computeTagSwitchMultiplier(observation, cameraName);
        linearStdDev  *= tagSwitchMultiplier;
        angularStdDev *= tagSwitchMultiplier;

        return EstimationResult.accepted(new VisionMeasurement(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
    }

    /**
     * Checks whether a pose observation should be rejected and returns the reason.
     * <p>
     * When {@code skipOdometryDeviation} is true, the odometry deviation filter is omitted so the estimator can accept poses regardless of how far
     * they are from the current odometry estimate. This is used during disabled-period pose calibration. All other safety filters (tag count,
     * ambiguity, field bounds, Z height, tag distance) always run.
     * </p>
     *
     * @param observation            the pose observation to evaluate
     * @param skipOdometryDeviation  true to bypass the odometry deviation check (calibration mode)
     * @return the rejection reason, or empty if the observation passes all checks
     */
    private Optional<RejectionReason> checkRejection(
            PoseObservation observation,
            boolean skipOdometryDeviation,
            String cameraName) {
        // If we see no tags, there is no useful pose to trust.
        if (observation.tagCount() == 0) {
            return Optional.of(RejectionReason.NO_TAGS);
        }

        // Reject observations where all tags are in the ignore list.
        if (ignoredTagIdSet.size() > 0 && observation.tagIds().length > 0) {
            boolean allIgnored = Arrays.stream(observation.tagIds())
                    .allMatch(ignoredTagIdSet::contains);
            if (allIgnored) {
                return Optional.of(RejectionReason.IGNORED_TAGS);
            }
        }

        // PhotonVision reports negative ambiguity for invalid single-tag ambiguity, so reject it
        // the same way we reject over-threshold ambiguity.
        if (observation.tagCount() == 1
                && (observation.ambiguity() < 0.0 || observation.ambiguity() > params.maxAmbiguity())) {
            return Optional.of(RejectionReason.SINGLE_TAG_AMBIGUITY);
        }

        // Multi-tag measurements can also be ambiguous with degenerate configurations.
        if (observation.tagCount() > 1
                && observation.ambiguity() > params.maxMultiTagAmbiguity()) {
            return Optional.of(RejectionReason.MULTI_TAG_AMBIGUITY);
        }

        // Reject observations where any single tag is too far away for reliable pose estimation.
        if (observation.maxTagDistance() > params.maxTagDistanceMeters()) {
            return Optional.of(RejectionReason.TOO_FAR);
        }

        // Reject poses with unreasonable Z heights.
        var pose = observation.pose();
        if (Math.abs(pose.getZ()) > params.maxZHeightMeters()) {
            return Optional.of(RejectionReason.BAD_Z_HEIGHT);
        }

        // Reject poses that fall outside the field rectangle.
        if (pose.getX() < 0.0 || pose.getX() > params.fieldLengthMeters()) {
            return Optional.of(RejectionReason.OUTSIDE_FIELD);
        }
        if (pose.getY() < 0.0 || pose.getY() > params.fieldWidthMeters()) {
            return Optional.of(RejectionReason.OUTSIDE_FIELD);
        }

        // Reject poses that deviate too far from odometry.
        // Skip this check during calibration mode, or during the initial acceptance window so the estimator can lock
        // onto a real field position before odometry has converged.
        if (!skipOdometryDeviation) {
            boolean withinInitialAcceptanceWindow = params.initialPoseAcceptanceCount() > 0
                    && acceptedMeasurementCount < params.initialPoseAcceptanceCount();
            if (!withinInitialAcceptanceWindow) {
                Pose2d odometryPose = odometryPoseSupplier.get();
                Pose2d visionPose2d = pose.toPose2d();
                double deviation    = odometryPose.getTranslation().getDistance(visionPose2d.getTranslation());
                lastOdometryDeviationMeters = deviation;
                if (deviation > params.maxPoseDeviationMeters()
                        && !isRecoveryMeasurementReady(observation, cameraName, visionPose2d)) {
                    return Optional.of(RejectionReason.ODOMETRY_DEVIATION);
                }
                if (deviation <= params.maxPoseDeviationMeters()) {
                    recoveryCandidatesByCamera.remove(cameraName);
                }
            }
        }

        return Optional.empty();
    }

    /**
     * Checks whether repeated high-quality multi-tag poses can safely recover from wheel-slip odometry drift.
     *
     * @param observation raw vision observation being evaluated
     * @param cameraName  camera that produced the observation
     * @param visionPose  field-relative vision pose in meters and radians
     * @return true when enough consecutive recovery candidates agree with each other
     */
    private boolean isRecoveryMeasurementReady(
            PoseObservation observation,
            String cameraName,
            Pose2d visionPose) {
        if (observation.tagCount() < params.recoveryMinimumTagCount()
                || params.recoveryRequiredConsecutiveFrames() <= 0) {
            recoveryCandidatesByCamera.remove(cameraName);
            return false;
        }

        RecoveryCandidate previousCandidate = recoveryCandidatesByCamera.get(cameraName);
        boolean consistentWithPrevious       = previousCandidate != null
                && previousCandidate.pose().getTranslation().getDistance(visionPose.getTranslation())
                        <= params.recoveryMaxTranslationMeters()
                && Math.abs(previousCandidate.pose().getRotation().minus(visionPose.getRotation()).getRadians())
                        <= params.recoveryMaxRotationRadians();

        int candidateCount = consistentWithPrevious ? previousCandidate.count() + 1 : 1;
        recoveryCandidatesByCamera.put(cameraName, new RecoveryCandidate(visionPose, candidateCount));
        lastRecoveryCandidateCount = candidateCount;

        if (candidateCount < params.recoveryRequiredConsecutiveFrames()) {
            return false;
        }

        lastMeasurementUsedRecovery = true;
        return true;
    }

    /**
     * Computes the standard deviation multiplier for tag-switch dampening.
     * <p>
     * Tracks the last accepted tag IDs per camera. When the set of tag IDs changes, returns the configured multiplier to reduce trust in the
     * transitional frame. When the tags are the same as the previous frame (or this is the first frame for a camera), returns 1.0.
     * </p>
     *
     * @param observation the current observation with tag IDs
     * @param cameraName  the camera that produced the observation
     * @return multiplier to apply to standard deviations (1.0 = no dampening)
     */
    private double computeTagSwitchMultiplier(PoseObservation observation, String cameraName) {
        // tagIds may be null during log replay if the field type is unsupported by AdvantageKit.
        if (observation.tagIds() == null) {
            return 1.0;
        }

        Set<Integer> currentTagIds  = Arrays.stream(observation.tagIds())
                .boxed()
                .collect(Collectors.toSet());

        Set<Integer> previousTagIds = lastTagIdsByCamera.put(cameraName, currentTagIds);

        // First observation from this camera — no dampening.
        if (previousTagIds == null) {
            return 1.0;
        }

        // Tags are the same as last time — no dampening.
        if (currentTagIds.equals(previousTagIds)) {
            return 1.0;
        }

        return params.tagSwitchStdDevMultiplier();
    }
}
