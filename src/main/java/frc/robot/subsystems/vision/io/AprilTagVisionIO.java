package frc.robot.subsystems.vision.io;

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
        public boolean connected = false;

        /**
         * Latest target yaw/pitch observation from the camera.
         */
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);

        /**
         * Pose estimates derived from current AprilTag observations.
         */
        public PoseObservation[] poseObservations = new PoseObservation[0];

        /**
         * IDs of tags observed in the most recent update.
         */
        public int[] tagIds = new int[0];
    }

    /**
     * Represents the angle to a target.
     */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /**
     * Represents a robot pose sample used for pose estimation.
     */
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance) {}

    /**
     * Refreshes the inputs structure with the latest state from the vision subsystem.
     *
     * @param inputs mutable inputs container to populate for logging
     */
    void updateInputs(AprilTagVisionIOInputs inputs);

}
