package frc.robot.subsystems.vision.io;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * AprilTagVisionIO implementation to use during simulation.
 */
public class AprilTagVisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {

    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;

    /**
     * Construct the instance with the usual info from config PLUS a pose
     * supplier which will likely be driveBaseSubsystem::getPost.
     */
    public AprilTagVisionIOPhotonVisionSim(
            String cameraName,
            Transform3d robotToCamera,
            AprilTagFieldLayout fieldLayout,
            Supplier<Pose2d> poseSupplier) {
        super(cameraName, robotToCamera, fieldLayout);
        this.poseSupplier = poseSupplier;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(fieldLayout);
        }

        var cameraProperties = new SimCameraProperties();
        var cameraSim = new PhotonCameraSim(camera, cameraProperties, fieldLayout);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
