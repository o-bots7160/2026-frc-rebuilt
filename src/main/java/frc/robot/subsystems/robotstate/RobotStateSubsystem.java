package frc.robot.subsystems.robotstate;

import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shared.subsystems.AbstractSubsystem;
import frc.robot.subsystems.robotstate.config.RobotStateSubsystemConfig;

/**
 * Centralized pose tracking subsystem that fuses odometry and vision into a single robot pose.
 * <p>
 * Use this subsystem as the authoritative source of field pose so commands and other subsystems do not need to combine multiple sensor sources on
 * their own.
 * </p>
 */
public class RobotStateSubsystem extends AbstractSubsystem<RobotStateSubsystemConfig> {

    private final Field2d    fieldDisplay                   = new Field2d();

    private Consumer<Pose2d> odometryResetConsumer          = pose -> {
                                                            };

    private Pose2d           odometryPose                   = new Pose2d();

    private Pose2d           estimatedPose                  = new Pose2d();

    private Pose2d           lastVisionPose                 = new Pose2d();

    private double           lastVisionTimestampSeconds     = Double.NaN;

    private boolean          hasVisionMeasurement           = false;

    private boolean          enableVisionFusion             = true;

    private double           visionBlendFactor              = 0.5;

    private double           visionMeasurementMaxAgeSeconds = 0.0;

    /**
     * Creates the Robot State subsystem.
     * <p>
     * This subsystem does not own hardware, but it still follows the command-based lifecycle for consistent updates and logging.
     * </p>
     *
     * @param config configuration values for pose fusion and logging
     */
    public RobotStateSubsystem(RobotStateSubsystemConfig config) {
        super(config);

        refreshTunables();
        SmartDashboard.putData("Field", fieldDisplay);
    }

    /**
     * Updates the internal pose estimate and logs telemetry each robot loop.
     * <p>
     * This method blends odometry with the most recent vision measurement when fusion is enabled and the measurement is fresh.
     * </p>
     */
    @Override
    public void periodic() {
        if (isSubsystemDisabled()) {
            return;
        }

        if (!isFMSAttached()) {
            refreshTunables();
        }

        updateEstimatedPose();
        log.recordOutput("Pose/Odometry", odometryPose);
        log.recordOutput("Pose/Estimated", estimatedPose);
        log.recordOutput("Pose/Vision", lastVisionPose);
        log.recordOutput("Vision/HasMeasurement", hasVisionMeasurement);
        log.recordOutput("Vision/TimestampSeconds", lastVisionTimestampSeconds);
        fieldDisplay.setRobotPose(estimatedPose);
    }

    /**
     * Registers a consumer that will reset odometry whenever the robot state pose is reset.
     * <p>
     * Use this to keep drivebase odometry aligned with the fused pose without tightly coupling the subsystems.
     * </p>
     *
     * @param consumer consumer that accepts the reset pose in meters and radians
     */
    public void setOdometryResetConsumer(Consumer<Pose2d> consumer) {
        if (consumer == null) {
            this.odometryResetConsumer = pose -> {
            };
            return;
        }

        this.odometryResetConsumer = consumer;
    }

    /**
     * Updates the odometry pose source for the robot state estimate.
     * <p>
     * Call this from the drivebase each loop so the estimator has the most recent encoder-based pose.
     * </p>
     *
     * @param pose latest odometry pose in meters and radians
     */
    public void updateOdometryPose(Pose2d pose) {
        if (isSubsystemDisabled()) {
            logDisabled("updateOdometryPose");
            return;
        }

        this.odometryPose = pose;
        if (!enableVisionFusion || !hasVisionMeasurement) {
            this.estimatedPose = pose;
        }
    }

    /**
     * Accepts a vision-based robot pose measurement for fusion.
     * <p>
     * The timestamp is used to reject stale measurements.
     * </p>
     *
     * @param robotPose        pose measurement in meters and radians
     * @param timestampSeconds timestamp of the measurement in seconds
     */
    public void addVisionMeasurement(
            Pose2d robotPose,
            double timestampSeconds) {
        if (isSubsystemDisabled()) {
            logDisabled("addVisionMeasurement");
            return;
        }

        this.lastVisionPose             = robotPose;
        this.lastVisionTimestampSeconds = timestampSeconds;
        this.hasVisionMeasurement       = true;
    }

    /**
     * Resets all pose tracking to the provided pose.
     * <p>
     * Use this at the start of autonomous or after localization resets so odometry and vision agree on the robot's position.
     * </p>
     *
     * @param pose pose to apply to odometry and the fused estimate
     */
    public void resetPose(Pose2d pose) {
        if (isSubsystemDisabled()) {
            logDisabled("resetPose");
            return;
        }

        this.odometryPose               = pose;
        this.estimatedPose              = pose;
        this.lastVisionPose             = pose;
        this.lastVisionTimestampSeconds = Double.NaN;
        this.hasVisionMeasurement       = false;

        odometryResetConsumer.accept(pose);
    }

    /**
     * Returns the current fused pose estimate.
     * <p>
     * Commands and subsystems should use this pose for field-relative decisions.
     * </p>
     *
     * @return latest fused pose in meters and radians
     */
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    private void updateEstimatedPose() {
        estimatedPose = odometryPose;

        if (!enableVisionFusion || !hasVisionMeasurement) {
            return;
        }

        double measurementAgeSeconds = Timer.getFPGATimestamp() - lastVisionTimestampSeconds;
        if (Double.isNaN(lastVisionTimestampSeconds)
                || measurementAgeSeconds > visionMeasurementMaxAgeSeconds) {
            return;
        }

        double clampedBlendFactor = MathUtil.clamp(visionBlendFactor, 0.0, 1.0);
        estimatedPose = estimatedPose.interpolate(lastVisionPose, clampedBlendFactor);
    }

    private void refreshTunables() {
        enableVisionFusion             = config.getEnableVisionFusion();
        visionBlendFactor              = config.getVisionBlendFactor();
        visionMeasurementMaxAgeSeconds = config.getVisionMeasurementMaxAgeSeconds();
    }
}
