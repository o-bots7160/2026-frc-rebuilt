package frc.robot.subsystems.indexer.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the indexer motor controller.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters inherited from {@link AbstractMotorConfig}. The indexer roller spins
 * continuously in both directions, so soft limits are typically not used. Changes require a redeploy.
 * </p>
 */
public class IndexerMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits motor fields and getters.
}
