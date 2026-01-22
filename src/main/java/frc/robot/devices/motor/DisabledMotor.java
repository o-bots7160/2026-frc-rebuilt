package frc.robot.devices.motor;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.devices.Motor;

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
     * @param voltageVolts desired voltage output in volts
     */
    @Override
    public void setVoltage(double voltageVolts) {
        lastCommandedVolts = voltageVolts;
    }

    /**
     * Ignores voltage requests and records the last value for telemetry.
     *
     * @param voltage desired voltage output
     */
    @Override
    public void setVoltage(Voltage voltage) {
        setVoltage(voltage.in(Units.Volts));
    }

    /**
     * Ignores duty-cycle requests and records the equivalent voltage.
     *
     * @param speed duty cycle from -1 (full reverse) to 1 (full forward)
     */
    @Override
    public void setSpeed(double speed) {
        setVoltage(speed * 12.0);
    }

    /**
     * Reports a neutral maximum target position of 0.
     *
     * @return maximum target position
     */
    @Override
    public double getMaximumTargetPosition() {
        return 0.0;
    }

    /**
     * Reports a neutral minimum target position of 0.
     *
     * @return minimum target position
     */
    @Override
    public double getMinimumTargetPosition() {
        return 0.0;
    }

    /**
     * Reports a neutral encoder position of 0.
     *
     * @return position in mechanism units
     */
    @Override
    public double getEncoderPosition() {
        return 0.0;
    }

    /**
     * Reports a neutral encoder velocity of 0.
     *
     * @return velocity in mechanism units per second
     */
    @Override
    public double getEncoderVelocity() {
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
     * Returns {@code null} because no controller exists when disabled.
     *
     * @return null SparkMax instance
     */
    @Override
    public SparkMax getMotor() {
        return null;
    }

    /**
     * Populates the telemetry inputs with neutral values.
     *
     * @param inputs mutable inputs container to fill for logging
     */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        inputs.positionRads            = 0.0;
        inputs.velocityRadPerSec       = 0.0;
        inputs.appliedVolts            = lastCommandedVolts;
        inputs.commandedVolts          = lastCommandedVolts;
        inputs.supplyCurrentAmps       = 0.0;
        inputs.temperatureCelsius      = 0.0;
        inputs.targetPositionRads      = Double.NaN;
        inputs.targetVelocityRadPerSec = Double.NaN;
    }
}
