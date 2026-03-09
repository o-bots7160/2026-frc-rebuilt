package frc.robot.subsystems.harvester.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the harvester arm motor controller.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters inherited from {@link AbstractMotorConfig}. The harvester arm uses soft limits
 * to prevent the arm from moving past its safe travel range. Changes require a redeploy.
 * </p>
 */
public class HarvesterMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits motor fields and getters.
}
