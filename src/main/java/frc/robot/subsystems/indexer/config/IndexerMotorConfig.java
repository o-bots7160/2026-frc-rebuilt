package frc.robot.subsystems.indexer.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the indexer motor controller.
 * <p>
 * Values are stored in degrees for soft limits and inherit tunable motor fields from {@link AbstractMotorConfig}. The indexer roller spins
 * continuously in both directions, so soft limits are typically not used.
 * </p>
 */
public class IndexerMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits tunable motor fields and suppliers.
}
