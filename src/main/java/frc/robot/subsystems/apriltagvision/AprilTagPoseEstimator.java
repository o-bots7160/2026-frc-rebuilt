package frc.robot.subsystems.apriltagvision;

import java.util.Optional;

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
     */
    public record Params(
            double fieldLengthMeters,
            double fieldWidthMeters,
            double maxAmbiguity,
            double linearStdDevBaseline,
            double angularStdDevBaseline) {
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
     * Single-tag PnP solves produce very noisy rotation estimates. By assigning an enormous angular
     * uncertainty the Kalman filter effectively ignores the rotation component while still fusing
     * the more reliable x/y translation.
     * </p>
     */
    private static final double SINGLE_TAG_ROTATION_STD_DEV_RADIANS = 1.0e6;

    private final Params params;

    /**
     * Creates a new AprilTagPoseEstimator.
     *
     * @param params configuration parameters for filtering and uncertainty
     */
    public AprilTagPoseEstimator(Params params) {
        this.params = params;
    }

    /**
     * Processes a pose observation and returns a measurement if accepted.
     * <p>
     * Standard deviations are scaled by distance squared divided by tag count, using meters for distance.
     * </p>
     *
     * @param observation the raw pose observation from vision
     * @return the validated measurement, or empty if the observation was rejected
     */
    public Optional<VisionMeasurement> estimate(PoseObservation observation) {
        // Stop early if the camera data fails our safety checks.
        if (shouldReject(observation)) {
            return Optional.empty();
        }

        // Farther tags and fewer tags mean less confidence, so scale up the uncertainty.
        double factor        = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev  = params.linearStdDevBaseline() * factor;
        double angularStdDev = observation.tagCount() > 1
                ? params.angularStdDevBaseline() * factor
                : SINGLE_TAG_ROTATION_STD_DEV_RADIANS;

        // Package the pose, timestamp, and uncertainty so the estimator can fuse it.
        return Optional.of(new VisionMeasurement(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
    }

    /**
     * Determines whether a pose observation should be rejected.
     * <p>
     * Observations are rejected if:
     * </p>
     * <ul>
     * <li>No tags were detected</li>
     * <li>Single-tag observation exceeds the dimensionless ambiguity threshold</li>
     * <li>Estimated pose is outside field boundaries in meters</li>
     * </ul>
     *
     * @param observation the pose observation to evaluate
     * @return true if the observation should be rejected
     */
    public boolean shouldReject(PoseObservation observation) {
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

        return false;
    }

}
