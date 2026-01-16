package frc.robot.devices.motor;

import org.littletonrobotics.junction.AutoLog;

/**
 * Defines the telemetry surface for a motor controller so AdvantageKit can mirror hardware readings.
 * <p>
 * Implementations should fill the {@link MotorIOInputs} container with raw measurements (in radians and seconds) once per loop.
 * Subsystems can then pass the generated {@code MotorIOInputsAutoLogged} into {@code Logger.processInputs(...)} for automatic logging.
 * </p>
 */
public interface MotorIO {

    /**
     * Container of motor state fields captured for telemetry. Units default to radians, seconds, Celsius, and Volts.
     */
    @AutoLog
    public static class MotorIOInputs {
        /** Measured mechanism position in radians (post conversion). */
        public double positionRads            = 0.0;

        /** Measured mechanism velocity in radians per second (post conversion). */
        public double velocityRadPerSec       = 0.0;

        /** Applied voltage from the motor controller output stage. */
        public double appliedVolts            = 0.0;

    /** Last commanded voltage request (open-loop), if applicable. */
    public double commandedVolts          = 0.0;

        /** Reported supply/phase current draw in amps. */
        public double supplyCurrentAmps       = 0.0;

        /** Motor temperature in degrees Celsius. */
        public double temperatureCelsius      = 0.0;

        /** Last commanded target position in radians, if applicable. */
        public double targetPositionRads      = Double.NaN;

        /** Last commanded target velocity in radians per second, if applicable. */
        public double targetVelocityRadPerSec = Double.NaN;
    }

    /**
     * Populates the inputs structure with the most recent readings from the motor controller.
     *
     * @param inputs mutable inputs container to fill for logging
     */
    void updateInputs(MotorIOInputs inputs);
}
