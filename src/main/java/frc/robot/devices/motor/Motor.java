package frc.robot.devices.motor;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;

/**
 * Minimal motor control surface for set-and-seek style mechanisms.
 * <p>
 * Positions and velocities are reported in radians so subsystems can work in standard WPILib math units. Callers should rely on the {@link MotorIO}
 * telemetry surfaces for logging and diagnostics.
 * </p>
 */
public interface Motor extends MotorIO {

    /**
     * Commands the motor with a voltage request.
     * <p>
     * Use this when closed-loop or feedforward logic has already computed the needed voltage. Implementations should apply the value immediately
     * without additional scaling.
     * </p>
     *
     * @param voltage desired output voltage, in volts
     */
    void setVoltage(Voltage voltage);

    /**
     * Commands an open-loop duty cycle.
     * <p>
     * This is intended for manual control or characterization when no closed-loop control is needed.
     * </p>
     *
     * @param speed duty cycle from -1 (full reverse) to 1 (full forward), unitless
     */
    void setSpeed(double speed);

    /**
     * Reports the current mechanism position.
     * <p>
     * The returned value should already include any gearing or offsets needed to express the mechanism position in radians.
     * </p>
     *
     * @return position in radians
     */
    double getPositionRadians();

    /**
     * Reports the current mechanism velocity.
     * <p>
     * The returned value should already include any gearing needed to express the mechanism velocity in radians per second.
     * </p>
     *
     * @return velocity in radians per second
     */
    double getVelocityRadiansPerSecond();

    /**
     * Reports the applied voltage.
     * <p>
     * Use this for logging or validation that the controller output matches expectations.
     * </p>
     *
     * @return voltage applied to the motor, in volts
     */
    Voltage getVoltage();

    /**
     * Stops the motor output.
     * <p>
     * Call this to halt motion immediately when the mechanism should idle or when disabling the subsystem.
     * </p>
     */
    void stop();

    /**
     * Exposes the underlying motor controller for advanced tuning.
     * <p>
     * Use this sparingly to access vendor-specific configuration that is not covered by this interface.
     * </p>
     *
     * @return wrapped {@link SparkMax} instance
     */
    SparkMax getMotor();
}
