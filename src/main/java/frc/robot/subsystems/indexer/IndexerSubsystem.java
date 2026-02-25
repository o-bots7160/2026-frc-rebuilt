package frc.robot.subsystems.indexer;

import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.indexer.config.IndexerSubsystemConfig;
import frc.robot.subsystems.indexer.devices.IndexerMotor;
import frc.robot.subsystems.indexer.devices.IndexerSimMotor;

/**
 * Indexer subsystem that controls when Fuel leaves the robot and enters the shooter.
 * <p>
 * The indexer sits directly upstream of the shooter and downstream of the feeder. It can spin forward to feed Fuel into the shooter flywheels, or
 * reverse to back Fuel toward the feeder. All RPM values in the public API represent roller (mechanism) speed after gear reduction, not motor shaft
 * speed.
 * </p>
 * <p>
 * The subsystem extends {@link AbstractVelocitySubsystem} which provides bidirectional velocity control. Positive RPM feeds forward (toward the
 * shooter); negative RPM feeds backward (toward the feeder).
 * </p>
 */
public class IndexerSubsystem extends AbstractVelocitySubsystem<IndexerSubsystemConfig> {

    /**
     * Builds the indexer subsystem with a single SparkMax-driven roller motor.
     *
     * @param config indexer configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public IndexerSubsystem(IndexerSubsystemConfig config) {
        super(config, buildVelocityMotor(config, config.indexerMotorConfig, IndexerMotor::create, IndexerSimMotor::create));
    }

    /**
     * Convenience method that sets the roller to reverse at the configured reverse velocity.
     * <p>
     * The configured reverse RPM is stored as a positive value; this method negates it so the roller spins backward toward the feeder.
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
