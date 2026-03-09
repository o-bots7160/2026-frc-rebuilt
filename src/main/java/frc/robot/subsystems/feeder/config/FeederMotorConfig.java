package frc.robot.subsystems.feeder.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the feeder belt motor controller.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters inherited from {@link AbstractMotorConfig}. The feeder belt spins continuously
 * in both directions, so soft limits are typically not used. Changes require a redeploy.
 * </p>
 */
public class FeederMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits motor fields and getters.
}
