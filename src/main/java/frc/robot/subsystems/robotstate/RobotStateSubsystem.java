package frc.robot.subsystems.robotstate;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shared.VisionMeasurementConsumer;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.robotstate.config.RobotStateSubsystemConfig;
import frc.robot.subsystems.robotstate.io.RobotStateIO;
import frc.robot.subsystems.robotstate.io.RobotStateIOInputsAutoLogged;

/**
 * Centralized pose tracking subsystem that serves as the single authority for robot field pose.
 * <p>
 * All pose sources (cameras, LIDAR, future sensors) funnel their measurements through this subsystem via {@link #addVisionMeasurement}. Fusion math
 * is delegated to the drivebase's internal YAGSL {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}, which provides Kalman-filter-based
 * fusion with latency compensation and uncertainty weighting. This subsystem reads the fused result each cycle and exposes it as the authoritative
 * pose for commands and other subsystems.
 * </p>
 */
public class RobotStateSubsystem extends AbstractSubsystem<RobotStateSubsystemConfig> {

    private final Field2d                      fieldDisplay               = new Field2d();

    private final RobotStateIO                 io;

    private final RobotStateIOInputsAutoLogged inputs                     = new RobotStateIOInputsAutoLogged();

    private final Supplier<Pose2d>             fusedPoseSupplier;

    private final Supplier<Pose2d>             odometryOnlyPoseSupplier;

    private final VisionMeasurementConsumer    visionForwarder;

    private final Consumer<Pose2d>             odometryResetConsumer;

    private Pose2d                             estimatedPose              = new Pose2d();

    private Pose2d                             odometryOnlyPose           = new Pose2d();

    private Pose2d                             lastVisionPose             = new Pose2d();

    private double                             lastVisionTimestampSeconds = Double.NaN;

    private boolean                            hasVisionMeasurement       = false;

    private boolean                            enableVisionFusion         = true;

    /**
     * Creates the Robot State subsystem with all cross-subsystem dependencies.
     * <p>
     * This subsystem does not own hardware, but it still follows the command-based lifecycle for consistent updates and logging. Dependencies are
     * injected here so every required wiring is enforced at compile time.
     * </p>
     *
     * @param config                   configuration values for vision fusion gating and logging
     * @param fusedPoseSupplier        supplier returning the Kalman-filtered pose from the drivebase in meters and radians
     * @param odometryOnlyPoseSupplier supplier returning the raw wheel+gyro pose without vision corrections in meters and radians
     * @param visionForwarder          consumer that forwards vision measurements to the drivebase's pose estimator
     * @param odometryResetConsumer    consumer that resets the drivebase odometry to a given pose in meters and radians
     */
    public RobotStateSubsystem(
            RobotStateSubsystemConfig config,
            Supplier<Pose2d> fusedPoseSupplier,
            Supplier<Pose2d> odometryOnlyPoseSupplier,
            VisionMeasurementConsumer visionForwarder,
            Consumer<Pose2d> odometryResetConsumer) {
        super(config);

        this.fusedPoseSupplier        = fusedPoseSupplier != null ? fusedPoseSupplier : Pose2d::new;
        this.odometryOnlyPoseSupplier = odometryOnlyPoseSupplier != null ? odometryOnlyPoseSupplier : Pose2d::new;
        this.visionForwarder          = visionForwarder != null ? visionForwarder : (pose, ts, sd) -> {
                                      };
        this.odometryResetConsumer    = odometryResetConsumer != null ? odometryResetConsumer : pose -> {
                                      };
        this.io                    = this::updateInputs;

        refreshTunables();
        SmartDashboard.putData("RobotStateSubsystem/Field", fieldDisplay);
    }

    /**
     * Reads the latest fused pose from the drivebase estimator and logs telemetry each robot loop.
     */
    @Override
    public void periodic() {
        if (isSubsystemDisabled()) {
            return;
        }

        if (!isFMSAttached()) {
            refreshTunables();
        }

        // Pull the latest fused pose from the drivebase's YAGSL estimator.
        estimatedPose    = fusedPoseSupplier.get();

        // Pull the raw odometry pose (wheel encoders + gyro only, no vision).
        odometryOnlyPose = odometryOnlyPoseSupplier.get();

        io.updateInputs(inputs);
        log.processInputs("RobotState", inputs);
        fieldDisplay.setRobotPose(estimatedPose);
    }

    /**
     * Accepts a vision-based robot pose measurement and forwards it to the drivebase's pose estimator for fusion.
     * <p>
     * All pose sources (cameras, LIDAR, etc.) should call this single method. The measurement is forwarded to the drivebase's YAGSL
     * {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} which handles Kalman-filter-based fusion, latency compensation, and uncertainty
     * weighting. When vision fusion is disabled in config, measurements are recorded for logging but not forwarded.
     * </p>
     *
     * @param robotPose          pose measurement in meters and radians
     * @param timestampSeconds   FPGA timestamp of the measurement in seconds
     * @param standardDeviations uncertainty in x (meters), y (meters), and rotation (radians)
     */
    public void addVisionMeasurement(
            Pose2d robotPose,
            double timestampSeconds,
            Matrix<N3, N1> standardDeviations) {
        if (isSubsystemDisabled()) {
            logDisabled("addVisionMeasurement");
            return;
        }

        // Always record the measurement for telemetry regardless of fusion state.
        this.lastVisionPose             = robotPose;
        this.lastVisionTimestampSeconds = timestampSeconds;
        this.hasVisionMeasurement       = true;

        if (!enableVisionFusion) {
            return;
        }

        // Delegate the actual fusion to the drivebase's YAGSL pose estimator.
        visionForwarder.accept(robotPose, timestampSeconds, standardDeviations);
    }

    /**
     * Resets all pose tracking to the provided pose.
     * <p>
     * Use this at the start of autonomous or after localization resets so odometry and vision agree on the robot's position. The reset is forwarded
     * to the drivebase so the internal YAGSL estimator is also reset.
     * </p>
     *
     * @param pose pose to apply to odometry and the fused estimate in meters and radians
     */
    public void resetPose(Pose2d pose) {
        if (isSubsystemDisabled()) {
            logDisabled("resetPose");
            return;
        }

        this.estimatedPose              = pose;
        this.lastVisionPose             = pose;
        this.lastVisionTimestampSeconds = Double.NaN;
        this.hasVisionMeasurement       = false;

        odometryResetConsumer.accept(pose);
    }

    /**
     * Resets the robot pose to the most recent vision measurement.
     * <p>
     * Call this at the start of a match to seed the pose estimator with the camera-derived position before switching to fused odometry-plus-vision
     * tracking. If no vision measurement has been received yet, the reset is skipped and a warning is logged so operators can diagnose camera issues.
     * </p>
     */
    public void resetPoseFromVision() {
        if (isSubsystemDisabled()) {
            logDisabled("resetPoseFromVision");
            return;
        }

        if (!hasVisionMeasurement) {
            reportWarning("resetPoseFromVision skipped: no vision measurement received yet.", false);
            return;
        }

        Pose2d poseFromVision = this.lastVisionPose;
        resetPose(poseFromVision);
    }

    /**
     * Returns the current fused pose estimate.
     * <p>
     * Commands and subsystems should use this pose for field-relative decisions. This is the single authoritative source of robot pose for the entire
     * codebase.
     * </p>
     *
     * @return latest fused pose in meters and radians
     */
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    private void updateInputs(RobotStateIO.RobotStateIOInputs inputs) {
        inputs.estimatedPose              = estimatedPose;
        inputs.odometryOnlyPose           = odometryOnlyPose;
        inputs.lastVisionPose             = lastVisionPose;
        inputs.lastVisionTimestampSeconds = lastVisionTimestampSeconds;
        inputs.hasVisionMeasurement       = hasVisionMeasurement;
        inputs.enableVisionFusion         = enableVisionFusion;
    }

    private void refreshTunables() {
        enableVisionFusion = config.getEnableVisionFusion();
    }
}
