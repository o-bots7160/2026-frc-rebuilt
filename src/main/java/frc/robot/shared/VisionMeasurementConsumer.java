package frc.robot.shared;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Functional interface for accepting a vision-based robot pose measurement along with its uncertainty.
 * <p>
 * Use this to decouple vision producers (cameras, LIDAR, etc.) from the pose fusion consumer so new pose sources can
 * be added with a single wiring change in {@code RobotContainer}.
 * </p>
 */
@FunctionalInterface
public interface VisionMeasurementConsumer {

    /**
     * Accepts a vision pose measurement for fusion into the robot pose estimate.
     *
     * @param pose               measured robot pose in meters and radians
     * @param timestampSeconds   FPGA timestamp of the measurement in seconds
     * @param standardDeviations uncertainty in x (meters), y (meters), and rotation (radians)
     */
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
}
