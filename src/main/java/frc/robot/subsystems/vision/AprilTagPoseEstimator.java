package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.io.AprilTagVisionIO.PoseObservation;

/**
 * Processes AprilTag pose observations and produces vision measurements
 * suitable for pose estimation.
 *
 * <p>This class encapsulates the filtering and standard deviation calculation
 * logic, making it testable without requiring PhotonVision, subsystem
 * infrastructure, or AdvantageKit logging.</p>
 */
public class AprilTagPoseEstimator {

    /**
     * Parameters for pose estimation filtering and uncertainty calculation.
     *
     * @param fieldLengthMeters       field length for bounds checking
     * @param fieldWidthMeters        field width for bounds checking
     * @param maxAmbiguity            maximum ambiguity for single-tag observations
     * @param linearStdDevBaseline    baseline std dev for x/y at 1m with 1 tag
     * @param angularStdDevBaseline   baseline std dev for rotation at 1m with 1 tag
     */
    public record Params(
            double fieldLengthMeters,
            double fieldWidthMeters,
            DoubleSupplier maxAmbiguity,
            DoubleSupplier linearStdDevBaseline,
            DoubleSupplier angularStdDevBaseline) {}

    /**
     * A validated vision measurement ready for pose estimation fusion.
     *
     * @param pose               the estimated robot pose
     * @param timestampSeconds   the timestamp when the observation was captured
     * @param standardDeviations uncertainty in x, y, and rotation
     */
    public record VisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            Matrix<N3, N1> standardDeviations) {}

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
     *
     * @param observation the raw pose observation from vision
     * @return the validated measurement, or empty if the observation was rejected
     */
    public Optional<VisionMeasurement> estimate(PoseObservation observation) {
        if (shouldReject(observation)) {
            return Optional.empty();
        }

        double factor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = params.linearStdDevBaseline().getAsDouble() * factor;
        double angularStdDev = params.angularStdDevBaseline().getAsDouble() * factor;

        return Optional.of(new VisionMeasurement(
                observation.pose().toPose2d(),
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
    }

    /**
     * Determines whether a pose observation should be rejected.
     *
     * <p>Observations are rejected if:</p>
     * <ul>
     *   <li>No tags were detected</li>
     *   <li>Single-tag observation exceeds the ambiguity threshold</li>
     *   <li>Estimated pose is outside field boundaries</li>
     * </ul>
     *
     * @param observation the pose observation to evaluate
     * @return true if the observation should be rejected
     */
    public boolean shouldReject(PoseObservation observation) {
        if (observation.tagCount() == 0) {
            return true;
        }

        if (observation.tagCount() == 1
                && observation.ambiguity() > params.maxAmbiguity().getAsDouble()) {
            return true;
        }

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
