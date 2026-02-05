package frc.robot.subsystems.robotstate;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
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

    private Pose2d         odometryPose                 = new Pose2d();

    private Pose2d         estimatedPose                = new Pose2d();

    private Pose2d         lastVisionPose               = new Pose2d();

    private Matrix<N3, N1> lastVisionStandardDeviations = VecBuilder.fill(0.0, 0.0, 0.0);

    private double         lastVisionTimestampSeconds   = Double.NaN;

    private boolean        hasVisionMeasurement         = false;

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
    }

    /**
     * Updates the internal pose estimate and logs telemetry each robot loop.
     * <p>
     * This method blends odometry with the most recent vision measurement when fusion is enabled and the measurement is fresh.
     * </p>
     */
    @Override
    public void periodic() {
        if (!isFMSAttached()) {
            refreshTunables();
        }

        if (isSubsystemDisabled()) {
            return;
        }

        updateEstimatedPose();
        log.recordOutput("Pose/Odometry", odometryPose);
        log.recordOutput("Pose/Estimated", estimatedPose);
        log.recordOutput("Pose/Vision", lastVisionPose);
        log.recordOutput("Vision/HasMeasurement", hasVisionMeasurement);
        log.recordOutput("Vision/TimestampSeconds", lastVisionTimestampSeconds);
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
        if (!config.getEnableVisionFusion() || !hasVisionMeasurement) {
            this.estimatedPose = pose;
        }
    }

    /**
     * Accepts a vision-based robot pose measurement for fusion.
     * <p>
     * The timestamp is used to reject stale measurements. Standard deviations are stored for telemetry and future estimator upgrades.
     * </p>
     *
     * @param robotPose          pose measurement in meters and radians
     * @param timestampSeconds   timestamp of the measurement in seconds
     * @param standardDeviations standard deviations for x/y/theta uncertainty
     */
    public void addVisionMeasurement(
            Pose2d robotPose,
            double timestampSeconds,
            Matrix<N3, N1> standardDeviations) {
        if (isSubsystemDisabled()) {
            logDisabled("addVisionMeasurement");
            return;
        }

        this.lastVisionPose               = robotPose;
        this.lastVisionTimestampSeconds   = timestampSeconds;
        this.lastVisionStandardDeviations = standardDeviations;
        this.hasVisionMeasurement         = true;
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

        this.odometryPose                 = pose;
        this.estimatedPose                = pose;
        this.lastVisionPose               = pose;
        this.lastVisionTimestampSeconds   = Double.NaN;
        this.lastVisionStandardDeviations = VecBuilder.fill(0.0, 0.0, 0.0);
        this.hasVisionMeasurement         = false;
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

    /**
     * Returns the most recent odometry pose.
     * <p>
     * Use this when you need raw encoder/gyro odometry without vision fusion.
     * </p>
     *
     * @return latest odometry pose in meters and radians
     */
    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    /**
     * Returns the most recent vision pose measurement.
     * <p>
     * This is useful for debugging camera alignment and measurement quality.
     * </p>
     *
     * @return latest vision pose in meters and radians
     */
    public Pose2d getLastVisionPose() {
        return lastVisionPose;
    }

    /**
     * Returns the timestamp of the latest vision measurement.
     * <p>
     * Use this to determine measurement age and diagnose camera latency.
     * </p>
     *
     * @return timestamp in seconds, or NaN if no measurement has been recorded
     */
    public double getLastVisionTimestampSeconds() {
        return lastVisionTimestampSeconds;
    }

    /**
     * Returns whether a vision measurement has been received.
     * <p>
     * This can be used to gate fusion logic or to display a vision status indicator.
     * </p>
     *
     * @return true if at least one vision measurement has been recorded
     */
    public boolean hasVisionMeasurement() {
        return hasVisionMeasurement;
    }

    /**
     * Returns the standard deviations for the most recent vision measurement.
     * <p>
     * These values represent uncertainty in x/y (meters) and rotation (radians).
     * </p>
     *
     * @return 3x1 matrix of standard deviations for the last vision measurement
     */
    public Matrix<N3, N1> getLastVisionStandardDeviations() {
        return lastVisionStandardDeviations;
    }

    private void updateEstimatedPose() {
        estimatedPose = odometryPose;

        if (!config.getEnableVisionFusion() || !hasVisionMeasurement) {
            return;
        }

        double measurementAgeSeconds = Timer.getFPGATimestamp() - lastVisionTimestampSeconds;
        if (Double.isNaN(lastVisionTimestampSeconds)
                || measurementAgeSeconds > config.getVisionMeasurementMaxAgeSeconds()) {
            return;
        }

        double blendFactor = clamp(config.getVisionBlendFactor(), 0.0, 1.0);
        estimatedPose = estimatedPose.interpolate(lastVisionPose, blendFactor);
    }

    private void refreshTunables() {
        config.getEnableVisionFusion();
        config.getVisionBlendFactor();
        config.getVisionMeasurementMaxAgeSeconds();
        config.getOdometryStandardDeviations();
        config.getVisionStandardDeviations();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
