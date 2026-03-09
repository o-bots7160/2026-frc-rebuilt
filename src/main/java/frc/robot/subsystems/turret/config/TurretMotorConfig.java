package frc.robot.subsystems.turret.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the turret motor controller and its travel limits.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters inherited from {@link AbstractMotorConfig}. Changes require a redeploy.
 * </p>
 */
public class TurretMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits motor fields and getters.
}
