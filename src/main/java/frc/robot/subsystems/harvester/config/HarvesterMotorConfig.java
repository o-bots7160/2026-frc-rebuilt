package frc.robot.subsystems.harvester.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the harvester arm motor controller.
 * <p>
 * Values are stored in degrees for soft limits and inherit tunable motor fields from {@link AbstractMotorConfig}. The harvester arm uses soft limits
 * to prevent the arm from moving past its safe travel range.
 * </p>
 */
public class HarvesterMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits tunable motor fields and suppliers.
}
