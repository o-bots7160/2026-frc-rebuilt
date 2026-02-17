package frc.robot.subsystems.climber.io;

import org.littletonrobotics.junction.AutoLog;

/**
 * Defines the telemetry surface for the climber's Time of Flight distance sensors so AdvantageKit can mirror hardware readings.
 * <p>
 * Implementations should fill the {@link ClimberSensorIOInputs} container with raw measurements once per loop. The subsystem then passes the
 * generated {@code ClimberSensorIOInputsAutoLogged} into {@code Logger.processInputs(...)} for automatic logging and replay support.
 * </p>
 */
public interface ClimberSensorIO {

    /**
     * Container of Time of Flight sensor state fields captured for telemetry. Distances are reported in millimeters.
     */
    @AutoLog
    public static class ClimberSensorIOInputs {
        /** Measured distance from the left Time of Flight sensor in millimeters. */
        public double leftDistanceMillimeters = 0.0;

        /** Measured distance from the right Time of Flight sensor in millimeters. */
        public double rightDistanceMillimeters = 0.0;

        /** Whether the left Time of Flight sensor is connected and returning valid readings. */
        public boolean leftSensorConnected = false;

        /** Whether the right Time of Flight sensor is connected and returning valid readings. */
        public boolean rightSensorConnected = false;
    }

    /**
     * Reads the latest distance measurements from both Time of Flight sensors and populates the inputs container.
     *
     * @param inputs container to fill with the latest sensor readings
     */
    void updateInputs(ClimberSensorIOInputs inputs);
}
