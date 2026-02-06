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
 * <p>
 * The pose supplier should provide raw odometry or ground-truth poses. Do not use the fused robot state pose to avoid feedback loops that can make
 * simulated vision look unrealistically perfect.
 * </p>
 */
public class AprilTagVisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {

    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;

    /**
     * Construct the instance with the usual config plus a live robot pose supplier.
     * <p>
    * Use the drive base pose so the simulated camera can render realistic tag observations.
     * </p>
     *
     * @param cameraName    name of the PhotonVision camera instance
     * @param robotToCamera robot-to-camera transform in meters and radians
     * @param fieldLayout   AprilTag field layout used for tag poses
    * @param poseSupplier  supplier of the robot pose used for simulation updates (odometry or ground truth, not fused robot state)
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
        var cameraSim        = new PhotonCameraSim(camera, cameraProperties, fieldLayout);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}
