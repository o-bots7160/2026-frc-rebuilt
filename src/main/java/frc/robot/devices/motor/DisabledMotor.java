package frc.robot.devices.motor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;

/**
 * No-op motor implementation used when a subsystem is disabled or has no hardware.
 * <p>
 * All commands are safely ignored, while getters return neutral values so callers can continue to run without null checks.
 * </p>
 */
public final class DisabledMotor implements Motor {

    private double lastCommandedVolts = 0.0;

    /**
     * Creates a disabled motor that ignores all output requests.
     */
    public DisabledMotor() {
    }

    /**
     * Ignores voltage requests and records the last value for telemetry.
     *
     * @param voltage desired voltage output
     */
    @Override
    public void setVoltage(Voltage voltage) {
        lastCommandedVolts = voltage.in(Units.Volts);
    }

    /**
     * Reports a neutral encoder position of 0.
     *
     * @return position in radians
     */
    @Override
    public double getPositionRadians() {
        return 0.0;
    }

    /**
     * Reports a neutral encoder velocity of 0.
     *
     * @return velocity in radians per second
     */
    @Override
    public double getVelocityRadiansPerSecond() {
        return 0.0;
    }

    /**
     * Reports the last commanded voltage as a neutral voltage measurement.
     *
     * @return voltage applied to the motor
     */
    @Override
    public Voltage getVoltage() {
        return Units.Volts.of(lastCommandedVolts);
    }

    /**
     * Stops output by clearing the stored voltage request.
     */
    @Override
    public void stop() {
        lastCommandedVolts = 0.0;
    }

    /**
     * Ignores encoder position changes because no hardware exists.
     *
     * @param positionRadians ignored
     */
    @Override
    public void setEncoderPosition(double positionRadians) {
        // No-op: no encoder to reset.
    }

    /**
     * Populates the telemetry inputs with neutral values.
     *
     * @param inputs mutable inputs container to fill for logging
     */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRadians         = 0.0;
        inputs.velocityRadPerSec       = 0.0;
        inputs.velocityMotorRpm        = 0.0;
        inputs.appliedVolts            = Units.Volts.of(lastCommandedVolts);
        inputs.commandedVolts          = Units.Volts.of(lastCommandedVolts);
        inputs.supplyCurrentAmps       = 0.0;
        inputs.temperatureCelsius      = 0.0;
        inputs.targetPositionRads      = Double.NaN;
        inputs.targetVelocityRadPerSec = Double.NaN;
    }
}
