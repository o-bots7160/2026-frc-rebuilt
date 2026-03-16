package frc.robot.subsystems.apriltagvision.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Defines the contract for vision sensor inputs that AdvantageKit will log.
 */
public interface AprilTagVisionIO {

    /**
     * Container of all vision telemetry fields that AdvantageKit will persist.
     */
    @AutoLog
    public static class AprilTagVisionIOInputs {
        /**
         * True when the camera is connected and reporting frames.
         */
        public boolean           connected               = false;

        /**
         * Latest target yaw/pitch observation from the camera.
         */
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

        /**
         * Pose estimates derived from current AprilTag observations.
         */
        public PoseObservation[] poseObservations        = new PoseObservation[0];

        /**
         * IDs of tags observed in the most recent update.
         */
        public int[]             tagIds                  = new int[0];
    }

    /**
     * Represents the yaw and pitch angles to a detected target.
     *
     * @param tx horizontal (yaw) angle from the camera center to the target
     * @param ty vertical (pitch) angle from the camera center to the target
     */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    /**
     * Represents a robot pose sample used for pose estimation.
     *
     * @param timestamp          FPGA timestamp in seconds when the observation was captured
     * @param pose               estimated 3D robot pose derived from the observed tags
     * @param ambiguity          pose ambiguity ratio from the solver (0 = unambiguous, 1 = fully ambiguous)
     * @param tagCount           number of AprilTags used to compute this pose estimate
     * @param averageTagDistance average distance in meters from the camera to the observed tags
     * @param tagIds             IDs of the AprilTags used to compute this pose estimate
     */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            int[] tagIds) {
    }

    /**
     * Refreshes the inputs structure with the latest state from the vision subsystem.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(AprilTagVisionIOInputs inputs);

}
