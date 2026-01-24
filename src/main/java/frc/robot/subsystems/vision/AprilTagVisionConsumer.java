package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Used to wire AprilTag Vision subsystem with the DriveBase subsystem.  The
 * Vision subsystem will provide an estimated robot pose to the consumer
 * based on its observations.
 *
 * <p>This interface is intentionally shaped to match
 * {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator#addVisionMeasurement
 * SwerveDrivePoseEstimator.addVisionMeasurement}, allowing vision measurements
 * to be passed through while keeping subsystems loosely coupled.</p>
 */
@FunctionalInterface
public interface AprilTagVisionConsumer {

    /**
     * Accepts a vision-based robot pose measurement for downstream consumption.
     *
     * <p>The method name {@code accept} follows the
     * {@link java.util.function.Consumer} convention, indicating that this
     * interface consumes a measurement but does not return a value or make
     * decisions about how the measurement is used.</p>
     *
     * <p>Each measurement includes a capture timestamp and a matrix of
     * standard deviations. The standard deviations are expected to increase
     * with target ambiguity and distance, causing the consumer to trust the
     * measurement less during sensor fusion.</p>
     *
     * @param robotPose the estimated robot pose derived from vision measurements
     * @param timestamp the timestamp (in seconds) when the image was captured
     *                  or when the measurement is valid
     * @param standardDeviations a 3x1 matrix of standard deviations representing
     *                           uncertainty in x, y, and rotation
     */
    public void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> standardDeviations);

}
