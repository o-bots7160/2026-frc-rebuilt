package frc.robot.subsystems.hopper.config;

import frc.robot.shared.config.AbstractMotorConfig;

/**
 * Configuration bundle for the hopper belt motor controller.
 * <p>
 * Values are stored in degrees for soft limits and inherit tunable motor fields from {@link AbstractMotorConfig}. For belt mechanisms that spin
 * continuously, soft limits are typically not used.
 * </p>
 */
public class HopperMotorConfig extends AbstractMotorConfig {
    // Intentionally empty; inherits tunable motor fields and suppliers.
}
