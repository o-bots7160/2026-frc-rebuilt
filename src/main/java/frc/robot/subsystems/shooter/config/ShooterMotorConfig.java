package frc.robot.subsystems.shooter.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the shooter motor controller.
 * <p>
 * Values are stored in degrees for soft limits and inherit tunable motor fields from {@link AbstractMotorConfig}. For flywheels, soft limits are
 * typically not used since the mechanism spins continuously.
 * </p>
 */
public class ShooterMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits tunable motor fields and suppliers.
}
