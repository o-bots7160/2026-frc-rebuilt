package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.vision.config.AprilTagVisionSubsystemConfig;
import frc.robot.subsystems.vision.io.AprilTagVisionIO;
import frc.robot.subsystems.vision.io.AprilTagVisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.io.AprilTagVisionIOPhotonVision;
import frc.robot.subsystems.vision.io.AprilTagVisionIOPhotonVisionSim;

/**
 * Subsystem that processes AprilTag camera observations for robot pose estimation.
 * <p>
 * Uses AprilTag detection via PhotonVision to provide pose corrections for the drivebase estimator. Pose filtering and uncertainty calculation are
 * delegated to {@link AprilTagPoseEstimator}.
 * </p>
 */
public class AprilTagVisionSubsystem extends AbstractSubsystem<AprilTagVisionSubsystemConfig> {

    private record CameraInstance(
            String name,
            AprilTagVisionIO io,
            AprilTagVisionIOInputsAutoLogged inputs,
            Alert disconnectedAlert) {
    }

    private final Map<String, CameraInstance> cameras;

    private final AprilTagVisionConsumer      consumer;

    private final AprilTagFieldLayout         fieldLayout;

    private final AprilTagPoseEstimator       poseEstimator;

    private boolean                           disabledPeriodicLogged;

    /**
     * Creates a new AprilTagVisionSubsystem.
     * <p>
     * Call this once during robot setup. The subsystem owns the camera I/O objects and streams accepted measurements to the consumer.
     * </p>
     *
     * @param config       configuration for vision processing and tunable thresholds
     * @param fieldLayout  AprilTag field layout in meters
     * @param consumer     consumer that receives accepted pose measurements in meters and radians
     * @param poseSupplier supplier for the current robot pose in meters and radians (used for simulation)
     */
    public AprilTagVisionSubsystem(
            AprilTagVisionSubsystemConfig config,
            AprilTagFieldLayout fieldLayout,
            AprilTagVisionConsumer consumer,
            Supplier<Pose2d> poseSupplier) {

        super(config);
        this.consumer      = consumer;
        this.fieldLayout   = fieldLayout;

        this.poseEstimator = new AprilTagPoseEstimator(new AprilTagPoseEstimator.Params(
                fieldLayout.getFieldLength(),
                fieldLayout.getFieldWidth(),
                config.getMaximumAmbiguity(),
                config.getLinearStandardDeviationBaseline(),
                config.getAngularStandardDeviationBaseline()));

        this.cameras       = createCameras(config, fieldLayout, poseSupplier);

        log.info("Initialized with " + cameras.size() + " camera(s)");
    }

    @Override
    public void periodic() {
        if (isSubsystemDisabled()) {
            if (!disabledPeriodicLogged) {
                log.warning("AprilTagVisionSubsystem periodic skipped: subsystem is disabled.");
                disabledPeriodicLogged = true;
            }
            return;
        }

        disabledPeriodicLogged = false;

        // Create per-cycle buckets so we can log everything together at the end of this loop.
        List<Pose3d> allTagPoses      = new ArrayList<>();
        List<Pose3d> allRobotPoses    = new ArrayList<>();
        List<Pose3d> allAcceptedPoses = new ArrayList<>();
        List<Pose3d> allRejectedPoses = new ArrayList<>();

        for (var camera : cameras.values()) {
            // Pull the newest data from this camera and send raw inputs to the logger.
            camera.io().updateInputs(camera.inputs());
            log.processInputs("Camera/" + camera.name(), camera.inputs());

            // Update the driver alert so we can see missing cameras quickly on the DS.
            camera.disconnectedAlert().set(!camera.inputs().connected);

            // Filter and forward any valid robot pose observations.
            processCameraObservations(
                    camera.name(),
                    camera.inputs(),
                    allTagPoses,
                    allRobotPoses,
                    allAcceptedPoses,
                    allRejectedPoses);
        }

        // Log a combined summary across all cameras for quick debugging.
        logSummary(allTagPoses, allRobotPoses, allAcceptedPoses, allRejectedPoses);
    }

    private Map<String, CameraInstance> createCameras(
            AprilTagVisionSubsystemConfig config,
            AprilTagFieldLayout fieldLayout,
            Supplier<Pose2d> poseSupplier) {

        var configCameras = config.getCameras();
        if (configCameras.isEmpty()) {
            log.warning("No cameras configured in VisionSubsystemConfig");
            return Map.of();
        }

        // Build camera objects in a stable order so logs are predictable.
        Map<String, CameraInstance> cameraMap = new LinkedHashMap<>(configCameras.size());
        for (var configEntry : configCameras.entrySet().stream()
                .sorted(Map.Entry.comparingByKey())
                .toList()) {
            String           cameraName  = configEntry.getKey();
            var              transform3d = configEntry.getValue().toTransform3d();

            AprilTagVisionIO visionIo;
            if (isSimulation()) {
                // In sim, use a fake camera that reads the provided robot pose supplier.
                visionIo = new AprilTagVisionIOPhotonVisionSim(
                        cameraName,
                        transform3d,
                        fieldLayout,
                        poseSupplier);
                log.info("Created simulated camera: " + cameraName);
            } else {
                // On the real robot, connect to PhotonVision for live AprilTag data.
                visionIo = new AprilTagVisionIOPhotonVision(
                        cameraName,
                        transform3d,
                        fieldLayout);
                log.info("Created real camera: " + cameraName);
            }

            var inputs            = new AprilTagVisionIOInputsAutoLogged();
            // Alert stays active when we stop receiving frames from this camera.
            var disconnectedAlert = new Alert(
                    "Vision camera '" + cameraName + "' is disconnected.",
                    AlertType.kWarning);

            cameraMap.put(cameraName, new CameraInstance(cameraName, visionIo, inputs, disconnectedAlert));
        }

        return Collections.unmodifiableMap(cameraMap);
    }

    private void processCameraObservations(
            String cameraName,
            AprilTagVisionIOInputsAutoLogged cameraInputs,
            List<Pose3d> allTagPoses,
            List<Pose3d> allRobotPoses,
            List<Pose3d> allAcceptedPoses,
            List<Pose3d> allRejectedPoses) {

        List<Pose3d> tagPoses      = new ArrayList<>();
        List<Pose3d> robotPoses    = new ArrayList<>();
        List<Pose3d> acceptedPoses = new ArrayList<>();
        List<Pose3d> rejectedPoses = new ArrayList<>();

        // Convert the observed tag IDs into field poses for logging.
        collectTagPoses(cameraInputs.tagIds, tagPoses);

        for (var poseObservation : cameraInputs.poseObservations) {
            // Track every raw robot pose we saw, even if we reject it later.
            robotPoses.add(poseObservation.pose());

            // Let the estimator decide if this observation is trustworthy.
            var maybeMeasurement = poseEstimator.estimate(poseObservation);
            if (maybeMeasurement.isEmpty()) {
                // Store rejected poses so we can diagnose filtering issues.
                rejectedPoses.add(poseObservation.pose());
                continue;
            }

            // Forward accepted measurements to the drivebase estimator.
            acceptedPoses.add(poseObservation.pose());
            var measurement = maybeMeasurement.get();
            consumer.accept(
                    measurement.pose(),
                    measurement.timestampSeconds(),
                    measurement.standardDeviations());
        }

        // Log per-camera data so we can compare cameras side by side.
        logCameraData(cameraName, tagPoses, robotPoses, acceptedPoses, rejectedPoses);

        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allAcceptedPoses.addAll(acceptedPoses);
        allRejectedPoses.addAll(rejectedPoses);
    }

    private void collectTagPoses(int[] tagIds, List<Pose3d> tagPoses) {
        // Look up each tag ID in the field layout and add its 3D pose if it exists.
        for (int tagId : tagIds) {
            fieldLayout.getTagPose(tagId).ifPresent(tagPoses::add);
        }
    }

    private void logCameraData(
            String cameraName,
            List<Pose3d> tagPoses,
            List<Pose3d> robotPoses,
            List<Pose3d> acceptedPoses,
            List<Pose3d> rejectedPoses) {
        // Prefix keeps the log tree organized per camera.
        String prefix = "Camera/" + cameraName;
        log.recordOutput(prefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        log.recordOutput(prefix + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        log.recordOutput(prefix + "/RobotPosesAccepted", acceptedPoses.toArray(new Pose3d[0]));
        log.recordOutput(prefix + "/RobotPosesRejected", rejectedPoses.toArray(new Pose3d[0]));
    }

    private void logSummary(
            List<Pose3d> tagPoses,
            List<Pose3d> robotPoses,
            List<Pose3d> acceptedPoses,
            List<Pose3d> rejectedPoses) {
        // Summary logs help spot system-wide issues without checking each camera.
        log.recordOutput("Summary/TagPoses", tagPoses.toArray(new Pose3d[0]));
        log.recordOutput("Summary/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        log.recordOutput("Summary/RobotPosesAccepted", acceptedPoses.toArray(new Pose3d[0]));
        log.recordOutput("Summary/RobotPosesRejected", rejectedPoses.toArray(new Pose3d[0]));
    }

}
