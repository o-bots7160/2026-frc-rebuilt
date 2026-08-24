package frc.robot.subsystems.apriltagvision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.apriltagvision.AprilTagPoseEstimator.Params;
import frc.robot.subsystems.apriltagvision.AprilTagPoseEstimator.RejectionReason;
import frc.robot.subsystems.apriltagvision.io.AprilTagVisionIO.PoseObservation;

/**
 * Tests vision filtering and recovery from large odometry errors.
 */
class AprilTagPoseEstimatorTest {

    private static final String CAMERA_NAME = "testCamera";

    @Test
    void acceptsConsistentMultiTagRecoveryAfterRequiredFrames() {
        AprilTagPoseEstimator estimator = createEstimator();

        assertOdometryDeviationRejected(estimator.estimate(observation(3.00, 2), CAMERA_NAME));
        assertEquals(1, estimator.getLastRecoveryCandidateCount());

        assertOdometryDeviationRejected(estimator.estimate(observation(3.08, 2), CAMERA_NAME));
        assertEquals(2, estimator.getLastRecoveryCandidateCount());

        var recoveredResult = estimator.estimate(observation(3.02, 2), CAMERA_NAME);

        assertTrue(recoveredResult.measurement().isPresent());
        assertTrue(estimator.didLastMeasurementUseRecovery());
        assertEquals(3, estimator.getLastRecoveryCandidateCount());
    }

    @Test
    void restartsRecoverySequenceWhenVisionPosesDisagree() {
        AprilTagPoseEstimator estimator = createEstimator();

        assertOdometryDeviationRejected(estimator.estimate(observation(3.00, 2), CAMERA_NAME));
        assertOdometryDeviationRejected(estimator.estimate(observation(3.08, 2), CAMERA_NAME));
        assertOdometryDeviationRejected(estimator.estimate(observation(4.00, 2), CAMERA_NAME));

        assertEquals(1, estimator.getLastRecoveryCandidateCount());
        assertFalse(estimator.didLastMeasurementUseRecovery());
    }

    @Test
    void neverUsesSingleTagPoseForRecovery() {
        AprilTagPoseEstimator estimator = createEstimator();

        for (int frame = 0; frame < 5; frame++) {
            assertOdometryDeviationRejected(estimator.estimate(observation(3.00, 1), CAMERA_NAME));
        }

        assertEquals(0, estimator.getLastRecoveryCandidateCount());
        assertFalse(estimator.didLastMeasurementUseRecovery());
    }

    private AprilTagPoseEstimator createEstimator() {
        Params params = new Params(
                16.0,
                8.0,
                0.15,
                0.03,
                1.0,
                5.0,
                1.5,
                0.2,
                0.5,
                new int[0],
                3.0,
                0,
                2,
                3,
                0.25,
                Math.toRadians(10.0));
        return new AprilTagPoseEstimator(params, Pose2d::new);
    }

    private PoseObservation observation(double xMeters, int tagCount) {
        int[] tagIds = tagCount == 1 ? new int[] { 1 } : new int[] { 1, 2 };
        return new PoseObservation(
                1.0,
                new Pose3d(xMeters, 2.0, 0.0, new Rotation3d()),
                0.05,
                tagCount,
                2.0,
                2.0,
                tagIds);
    }

    private void assertOdometryDeviationRejected(AprilTagPoseEstimator.EstimationResult result) {
        assertEquals(RejectionReason.ODOMETRY_DEVIATION, result.rejectionReason().orElseThrow());
        assertTrue(result.measurement().isEmpty());
    }
}
