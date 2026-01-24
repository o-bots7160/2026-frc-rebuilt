package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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
 *
 * <p>Uses AprilTag detection via PhotonVision to provide vision-based pose
 * corrections to the drivebase pose estimator. Pose filtering and uncertainty
 * calculation are delegated to {@link AprilTagPoseEstimator}.</p>
 */
public class AprilTagVisionSubsystem extends AbstractSubsystem<AprilTagVisionSubsystemConfig> {

    private record CameraInstance(
            String name,
            AprilTagVisionIO io,
            AprilTagVisionIOInputsAutoLogged inputs,
            Alert disconnectedAlert) {}

    private final Map<String, CameraInstance> cameras;
    private final AprilTagVisionConsumer consumer;
    private final AprilTagFieldLayout fieldLayout;
    private final AprilTagPoseEstimator poseEstimator;

    /**
     * Creates a new AprilTagVisionSubsystem.
     *
     * @param config       configuration for vision processing
     * @param fieldLayout  the AprilTag field layout
     * @param consumer     consumer that receives validated pose measurements
     * @param poseSupplier supplier for robot pose (used for simulation)
     */
    public AprilTagVisionSubsystem(
            AprilTagVisionSubsystemConfig config,
            AprilTagFieldLayout fieldLayout,
            AprilTagVisionConsumer consumer,
            Supplier<Pose2d> poseSupplier) {

        super(config);
        this.consumer = consumer;
        this.fieldLayout = fieldLayout;

        this.poseEstimator = new AprilTagPoseEstimator(new AprilTagPoseEstimator.Params(
                fieldLayout.getFieldLength(),
                fieldLayout.getFieldWidth(),
                config.getMaximumAmbiguity()::get,
                config.getLinearStandardDeviationBaseline()::get,
                config.getAngularStandardDeviationBaseline()::get));

        this.cameras = createCameras(config, fieldLayout, poseSupplier);

        log.info("AprilTagVisionSubsystem initialized with " + cameras.size() + " camera(s)");
    }

    @Override
    public void periodic() {
        if (isSubsystemDisabled()) {
            return;
        }

        List<Pose3d> allTagPoses      = new ArrayList<>();
        List<Pose3d> allRobotPoses    = new ArrayList<>();
        List<Pose3d> allAcceptedPoses = new ArrayList<>();
        List<Pose3d> allRejectedPoses = new ArrayList<>();

        for (var camera : cameras.values()) {
            camera.io().updateInputs(camera.inputs());
            Logger.processInputs("Vision/Camera/" + camera.name(), camera.inputs());

            camera.disconnectedAlert().set(!camera.inputs().connected);

            processCameraObservations(
                    camera.name(),
                    camera.inputs(),
                    allTagPoses,
                    allRobotPoses,
                    allAcceptedPoses,
                    allRejectedPoses);
        }

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

        Map<String, CameraInstance> cameraMap = new LinkedHashMap<>(configCameras.size());
        for (var configEntry : configCameras.entrySet().stream()
                .sorted(Map.Entry.comparingByKey())
                .toList()) {
            String cameraName = configEntry.getKey();
            var transform3d = configEntry.getValue().toTransform3d();

            AprilTagVisionIO visionIo;
            if (isSimulation()) {
                visionIo = new AprilTagVisionIOPhotonVisionSim(
                        cameraName,
                        transform3d,
                        fieldLayout,
                        poseSupplier);
                log.info("Created simulated camera: " + cameraName);
            } else {
                visionIo = new AprilTagVisionIOPhotonVision(
                        cameraName,
                        transform3d,
                        fieldLayout);
                log.info("Created real camera: " + cameraName);
            }

            var inputs = new AprilTagVisionIOInputsAutoLogged();
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

        collectTagPoses(cameraInputs.tagIds, tagPoses);

        for (var poseObservation : cameraInputs.poseObservations) {
            robotPoses.add(poseObservation.pose());

            var maybeMeasurement = poseEstimator.estimate(poseObservation);
            if (maybeMeasurement.isEmpty()) {
                rejectedPoses.add(poseObservation.pose());
                continue;
            }

            acceptedPoses.add(poseObservation.pose());
            var measurement = maybeMeasurement.get();
            consumer.accept(
                    measurement.pose(),
                    measurement.timestampSeconds(),
                    measurement.standardDeviations());
        }

        logCameraData(cameraName, tagPoses, robotPoses, acceptedPoses, rejectedPoses);

        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allAcceptedPoses.addAll(acceptedPoses);
        allRejectedPoses.addAll(rejectedPoses);
    }

    private void collectTagPoses(int[] tagIds, List<Pose3d> tagPoses) {
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

        String prefix = "Vision/Camera/" + cameraName;
        Logger.recordOutput(prefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(prefix + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(prefix + "/RobotPosesAccepted", acceptedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(prefix + "/RobotPosesRejected", rejectedPoses.toArray(new Pose3d[0]));
    }

    private void logSummary(
            List<Pose3d> tagPoses,
            List<Pose3d> robotPoses,
            List<Pose3d> acceptedPoses,
            List<Pose3d> rejectedPoses) {

        Logger.recordOutput("Vision/Summary/TagPoses", tagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", robotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", acceptedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", rejectedPoses.toArray(new Pose3d[0]));
    }

}
