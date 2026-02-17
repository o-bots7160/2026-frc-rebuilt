package frc.robot.subsystems.climber.config;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;

/**
 * Configuration bundle for the climber mechanism. Values are stored in degrees for readability but converted to radians at runtime where needed.
 * <p>
 * The climber extends the set-and-seek pattern to drive arms or hooks through staged climb positions. Motor configuration is a placeholder until
 * hardware is selected. Sensor configuration controls the two front-facing Time of Flight sensors used for tower alignment.
 * </p>
 */
public class ClimberSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Motor configuration bundle for the climber mechanism (placeholder until hardware is selected). */
    public ClimberMotorConfig climberMotorConfig = new ClimberMotorConfig();

    /** Time of Flight sensor configuration bundle for tower alignment. */
    public ClimberSensorConfig climberSensorConfig = new ClimberSensorConfig();
}
