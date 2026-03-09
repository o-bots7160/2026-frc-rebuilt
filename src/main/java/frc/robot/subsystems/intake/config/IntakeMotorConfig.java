package frc.robot.subsystems.intake.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the intake roller motor controller.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters inherited from {@link AbstractMotorConfig}. For roller mechanisms that spin
 * continuously, soft limits are typically not used. Changes require a redeploy.
 * </p>
 */
public class IntakeMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits motor fields and getters.
}
