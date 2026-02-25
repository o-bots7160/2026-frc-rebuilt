package frc.robot.subsystems.intake.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the intake roller motor controller.
 * <p>
 * Values are stored in degrees for soft limits and inherit tunable motor fields from {@link AbstractMotorConfig}. For roller mechanisms that spin
 * continuously, soft limits are typically not used.
 * </p>
 */
public class IntakeMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits tunable motor fields and suppliers.
}
