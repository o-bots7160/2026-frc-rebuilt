package frc.robot.subsystems.shooter.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the shooter motor controller.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters inherited from {@link AbstractMotorConfig}. For flywheels, soft limits are
 * typically not used since the mechanism spins continuously. Changes require a redeploy.
 * </p>
 */
public class ShooterMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits motor fields and getters.
}
