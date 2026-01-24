package frc.robot.subsystems.vision.io;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * AprilTagVisionIO implementation for PhotonVision cameras.
 *
 * <p>Processes camera frames to extract AprilTag observations for pose
 * estimation.</p>
 */
public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {

    protected final PhotonCamera        camera;

    protected final Transform3d         robotToCamera;

    protected final AprilTagFieldLayout fieldLayout;

    public AprilTagVisionIOPhotonVision(
            String cameraName,
            Transform3d robotToCamera,
            AprilTagFieldLayout fieldLayout) {
        this.camera        = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
        this.fieldLayout   = fieldLayout;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // neither of these are logged; they're collections built up based on
        // PhotonVision pipeline results and processed to log info
        Set<Integer> observedTagIds        = new HashSet<>();
        List<PoseObservation> observations = new ArrayList<>();

        for (var result : camera.getAllUnreadResults()) {

            // extractTargetObservation called even for older results because
            // maybe newest don't have targets
            inputs.latestTargetObservation = extractTargetObservation(result);

            Optional<PoseObservation> observation = extractPoseObservation(result, observedTagIds);
            observation.ifPresent(observations::add);
        }

        inputs.poseObservations = observations.toArray(new PoseObservation[0]);
        inputs.tagIds           = observedTagIds.stream().mapToInt(Integer::intValue).toArray();
    }

    /**
     * Extracts the yaw/pitch angles to the best visible target.
     */
    private TargetObservation extractTargetObservation(final PhotonPipelineResult pipelineResult) {
        if (!pipelineResult.hasTargets()) {
            return new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        }

        var photoTrackedTarget = pipelineResult.getBestTarget();
        return new TargetObservation(
                Rotation2d.fromDegrees(photoTrackedTarget.getYaw()),
                Rotation2d.fromDegrees(photoTrackedTarget.getPitch()));
    }

    /**
     * Extracts a pose observation from the frame, preferring multi-tag results when available.
     */
    private Optional<PoseObservation> extractPoseObservation(
            final PhotonPipelineResult pipelineResult,
            Set<Integer> observedTagIds) {

        if (pipelineResult.multitagResult.isPresent()) {
            return extractMultiTagObservation(pipelineResult, observedTagIds);
        }

        if (!pipelineResult.targets.isEmpty()) {
            return extractSingleTagObservation(pipelineResult, observedTagIds);
        }

        return Optional.empty();
    }

    /**
     * Processes a multi-tag result into a pose observation.
     */
    private Optional<PoseObservation> extractMultiTagObservation(
            final PhotonPipelineResult pipelineResult,
            Set<Integer> observedTagIds) {

        MultiTargetPNPResult multitagResult = pipelineResult.multitagResult.get();

        Pose3d robotPose          = calculateRobotPoseFromCamera(multitagResult.estimatedPose.best);
        double averageTagDistance = calculateAverageTagDistance(pipelineResult.targets);
        int tagCount              = multitagResult.fiducialIDsUsed.size();

        observedTagIds.addAll(multitagResult.fiducialIDsUsed.stream()
                .map(Short::intValue)
                .toList());

        return Optional.of(new PoseObservation(
                pipelineResult.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                tagCount,
                averageTagDistance));
    }

    /**
     * Processes a single-tag result into a pose observation.
     */
    private Optional<PoseObservation> extractSingleTagObservation(
            final PhotonPipelineResult pipelineResult,
            Set<Integer> observedTagIds) {

        PhotonTrackedTarget target = pipelineResult.targets.get(0);

        Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.fiducialId);
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d robotPose     = calculateRobotPoseFromTag(tagPose.get(), target.bestCameraToTarget);
        double tagDistance   = target.bestCameraToTarget.getTranslation().getNorm();

        observedTagIds.add(target.fiducialId);

        return Optional.of(new PoseObservation(
                pipelineResult.getTimestampSeconds(),
                robotPose,
                target.poseAmbiguity,
                1,
                tagDistance));
    }

    /**
     * Converts a field-to-camera transform into a robot pose.
     */
    private Pose3d calculateRobotPoseFromCamera(final Transform3d fieldToCamera) {
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        return new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
    }

    /**
     * Calculates robot pose given a known tag pose and camera-to-tag transform.
     */
    private Pose3d calculateRobotPoseFromTag(
            final Pose3d tagPose,
            final Transform3d cameraToTarget) {
        Transform3d fieldToTarget = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        return calculateRobotPoseFromCamera(fieldToCamera);
    }

    /**
     * Computes the average distance from camera to all visible tags.
     */
    private double calculateAverageTagDistance(final List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return 0.0;
        }

        double totalDistance = targets.stream()
                .mapToDouble(t -> t.bestCameraToTarget.getTranslation().getNorm())
                .sum();

        return totalDistance / targets.size();
    }
}
