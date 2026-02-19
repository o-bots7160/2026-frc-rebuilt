package frc.robot.subsystems.indexer;

import edu.wpi.first.math.util.Units;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.config.RobotEnvironment;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.indexer.config.IndexerSubsystemConfig;
import frc.robot.subsystems.indexer.devices.IndexerMotor;
import frc.robot.subsystems.indexer.devices.IndexerSimMotor;

/**
 * Indexer subsystem that controls when Fuel leaves the robot and enters the shooter.
 * <p>
 * The indexer sits directly upstream of the shooter and downstream of the feeder. It can spin forward to feed Fuel into the shooter flywheels, or
 * reverse to back Fuel toward the hopper. All RPM values in the public API represent roller (mechanism) speed after gear reduction, not motor shaft
 * speed.
 * </p>
 * <p>
 * The subsystem extends {@link AbstractVelocitySubsystem} which provides bidirectional velocity control. Positive RPM feeds forward (toward the
 * shooter); negative RPM feeds backward (toward the hopper).
 * </p>
 */
public class IndexerSubsystem extends AbstractVelocitySubsystem<IndexerSubsystemConfig> {

    /**
     * Builds the correct motor implementation for the current environment.
     * <p>
     * Returns null when the subsystem is disabled so the parent falls back to {@link frc.robot.devices.motor.DisabledMotor}.
     * </p>
     */
    private static Motor buildMotor(IndexerSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        return RobotEnvironment.isReal()
                ? IndexerMotor.create(config.indexerMotorConfig)
                : IndexerSimMotor.create(
                        config.indexerMotorConfig,
                        () -> rpmToDegreesPerSecond(config.getMaximumVelocityRpm()),
                        () -> rpmToDegreesPerSecond(config.getMaximumAccelerationRpmPerSecond()));
    }

    /**
     * Converts RPM to degrees per second.
     * <p>
     * Used internally to convert from the RPM-based config to the degrees-per-second suppliers expected by the simulation motor.
     * </p>
     *
     * @param rpm value in rotations per minute
     * @return equivalent value in degrees per second
     */
    private static double rpmToDegreesPerSecond(double rpm) {
        return Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    }

    /**
     * Builds the indexer subsystem with a single SparkMax-driven roller motor.
     *
     * @param config indexer configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public IndexerSubsystem(IndexerSubsystemConfig config) {
        this(config, buildMotor(config));
    }

    private IndexerSubsystem(IndexerSubsystemConfig config, Motor motor) {
        super(config, motor);
    }

    /**
     * Publishes indexer telemetry each cycle.
     */
    @Override
    public void periodic() {
        super.periodic();

        log.recordOutput("readyToFeed", isReadyToFeed());
        log.recordOutput("measuredRpm", getMeasuredVelocityRpm());
        log.recordOutput("targetRpm", getTargetVelocityRpm());
    }

    /**
     * Returns the indexer configuration bundle for command factories that need access to indexer-specific settings.
     *
     * @return indexer subsystem configuration
     */
    public IndexerSubsystemConfig getConfig() {
        return config;
    }

    /**
     * Reports whether the indexer roller is spinning fast enough and has been stable long enough to feed.
     * <p>
     * Autonomous and composite commands can wait for this signal before advancing to the next step.
     * </p>
     *
     * @return true when the roller is at the target RPM and ready to feed
     */
    public boolean isReadyToFeed() {
        return isAtTargetVelocity();
    }

    /**
     * Convenience method that sets the roller to reverse at the configured reverse velocity.
     * <p>
     * The configured reverse RPM is stored as a positive value; this method negates it so the roller spins backward toward the hopper.
     * </p>
     */
    public void setReverseVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("setReverseVelocity");
            return;
        }
        setTargetVelocityRpm(-config.getReverseVelocityRpm());
    }

    /**
     * Convenience method that sets the roller to the configured default feed velocity.
     */
    public void setFeedVelocity() {
        if (isSubsystemDisabled()) {
            logDisabled("setFeedVelocity");
            return;
        }
        setTargetVelocityRpm(config.getFeedVelocityRpm());
    }

}
