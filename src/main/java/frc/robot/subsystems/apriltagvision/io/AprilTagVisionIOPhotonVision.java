package frc.robot.subsystems.apriltagvision.io;

import java.util.ArrayList;
import java.util.Comparator;
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
 * <p>
 * Processes camera frames to extract AprilTag observations for pose estimation.
 * </p>
 */
public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {

    /**
     * Maximum number of pipeline results to process per robot loop cycle.
     * <p>
     * PhotonVision cameras can produce frames faster than the 50 Hz robot loop. When the robot
     * falls behind (e.g., during a GC pause or heavy logging cycle), {@code getAllUnreadResults()}
     * returns the entire backlog. Processing every frame in one cycle causes massive loop overruns.
     * Limiting to the two most recent results keeps the loop within budget while still capturing
     * multi-tag and single-tag observations from the latest frames.
     * </p>
     */
    private static final int MAX_RESULTS_PER_CYCLE = 2;

    /**
     * PhotonVision camera instance used to pull pipeline results.
     */
    protected final PhotonCamera        camera;

    /**
     * Transform from robot center to the camera position.
     */
    protected final Transform3d         robotToCamera;

    /**
     * Field layout that maps AprilTag IDs to field poses.
     */
    protected final AprilTagFieldLayout fieldLayout;

    /**
     * Maximum distance in meters from the camera to a tag before the tag is excluded from observations.
     */
    protected final double              maxTagDistanceMeters;

    /**
     * Creates a PhotonVision-backed AprilTag IO implementation.
     *
     * @param cameraName           name of the PhotonVision camera instance
     * @param robotToCamera        robot-to-camera transform in meters and radians
     * @param fieldLayout          AprilTag field layout used to look up tag poses
     * @param maxTagDistanceMeters maximum distance in meters from camera to a tag; tags beyond this are excluded
     */
    public AprilTagVisionIOPhotonVision(
            String cameraName,
            Transform3d robotToCamera,
            AprilTagFieldLayout fieldLayout,
            double maxTagDistanceMeters) {
        this.camera               = new PhotonCamera(cameraName);
        this.robotToCamera        = robotToCamera;
        this.fieldLayout          = fieldLayout;
        this.maxTagDistanceMeters = maxTagDistanceMeters;
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Neither of these are logged; they're collections built up based on
        // PhotonVision pipeline results and processed to log info.
        Set<Integer>          observedTagIds = new HashSet<>();
        List<PoseObservation> observations   = new ArrayList<>();

        List<PhotonPipelineResult> allResults = camera.getAllUnreadResults();

        // Only process the most recent results to avoid loop overruns when
        // a backlog of frames has accumulated since the last cycle.
        int startIndex = Math.max(0, allResults.size() - MAX_RESULTS_PER_CYCLE);

        for (int i = startIndex; i < allResults.size(); i++) {
            PhotonPipelineResult result = allResults.get(i);

            // extractTargetObservation called even for older results because
            // maybe newest don't have targets.
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
     * <p>
     * When a multi-tag result is present but one or more tags exceed the maximum distance, the
     * multi-tag PnP solution is discarded because far tags add noise to the solve. A single-tag
     * observation from the closest in-range target is produced instead.
     * </p>
     */
    private Optional<PoseObservation> extractPoseObservation(
            final PhotonPipelineResult pipelineResult,
            Set<Integer> observedTagIds) {

        if (pipelineResult.multitagResult.isPresent()) {
            boolean hasFarTag = pipelineResult.targets.stream()
                    .anyMatch(t -> t.bestCameraToTarget.getTranslation().getNorm() > maxTagDistanceMeters);

            if (!hasFarTag) {
                return extractMultiTagObservation(pipelineResult, observedTagIds);
            }

            // Far tags contaminate the multi-tag PnP solve. Fall back to closest in-range tag.
            return extractClosestInRangeObservation(pipelineResult, observedTagIds);
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

        MultiTargetPNPResult multitagResult     = pipelineResult.multitagResult.get();

        Pose3d               robotPose          = calculateRobotPoseFromCamera(multitagResult.estimatedPose.best);
        double               averageTagDistance = calculateAverageTagDistance(pipelineResult.targets);
        double               maxTagDistance     = calculateMaxTagDistance(pipelineResult.targets);
        int                  tagCount           = multitagResult.fiducialIDsUsed.size();

        observedTagIds.addAll(multitagResult.fiducialIDsUsed.stream()
                .map(Short::intValue)
                .toList());

        int[] tagIdArray = multitagResult.fiducialIDsUsed.stream()
                .mapToInt(Short::intValue)
                .toArray();

        return Optional.of(new PoseObservation(
                pipelineResult.getTimestampSeconds(),
                robotPose,
                multitagResult.estimatedPose.ambiguity,
                tagCount,
                averageTagDistance,
                maxTagDistance,
                tagIdArray));
    }

    /**
     * Processes a single-tag result into a pose observation.
     */
    private Optional<PoseObservation> extractSingleTagObservation(
            final PhotonPipelineResult pipelineResult,
            Set<Integer> observedTagIds) {

        PhotonTrackedTarget target  = pipelineResult.targets.get(0);

        Optional<Pose3d>    tagPose = fieldLayout.getTagPose(target.fiducialId);
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d robotPose   = calculateRobotPoseFromTag(tagPose.get(), target.bestCameraToTarget);
        double tagDistance = target.bestCameraToTarget.getTranslation().getNorm();

        observedTagIds.add(target.fiducialId);

        return Optional.of(new PoseObservation(
                pipelineResult.getTimestampSeconds(),
                robotPose,
                target.poseAmbiguity,
                1,
                tagDistance,
                tagDistance,
                new int[] { target.fiducialId }));
    }

    /**
     * Finds the closest target within the maximum distance and produces a single-tag observation.
     * <p>
     * All visible tag IDs are still added to the observed set for diagnostic logging, even though
     * only the closest in-range tag is used for the pose estimate.
     * </p>
     */
    private Optional<PoseObservation> extractClosestInRangeObservation(
            final PhotonPipelineResult pipelineResult,
            Set<Integer> observedTagIds) {

        // Record all visible tags for logging regardless of distance.
        pipelineResult.targets.forEach(t -> observedTagIds.add(t.fiducialId));

        Optional<PhotonTrackedTarget> closestTarget = pipelineResult.targets.stream()
                .filter(t -> t.bestCameraToTarget.getTranslation().getNorm() <= maxTagDistanceMeters)
                .min(Comparator.comparingDouble(t -> t.bestCameraToTarget.getTranslation().getNorm()));

        if (closestTarget.isEmpty()) {
            return Optional.empty();
        }

        PhotonTrackedTarget target  = closestTarget.get();
        Optional<Pose3d>    tagPose = fieldLayout.getTagPose(target.fiducialId);
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        Pose3d robotPose  = calculateRobotPoseFromTag(tagPose.get(), target.bestCameraToTarget);
        double tagDistance = target.bestCameraToTarget.getTranslation().getNorm();

        return Optional.of(new PoseObservation(
                pipelineResult.getTimestampSeconds(),
                robotPose,
                target.poseAmbiguity,
                1,
                tagDistance,
                tagDistance,
                new int[] { target.fiducialId }));
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

    /**
     * Computes the maximum distance from camera to any single visible tag.
     */
    private double calculateMaxTagDistance(final List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return 0.0;
        }

        return targets.stream()
                .mapToDouble(t -> t.bestCameraToTarget.getTranslation().getNorm())
                .max()
                .orElse(0.0);
    }
}
