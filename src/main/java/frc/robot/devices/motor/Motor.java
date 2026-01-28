package frc.robot.devices.motor;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;

/**
 * Minimal motor control surface for set-and-seek style mechanisms.
 * <p>
 * Positions and velocities are expressed in mechanism units so subsystems can work in familiar scales (for example, turret degrees). Implementations
 * should also provide {@link MotorIO} telemetry for logging.
 * </p>
 */
public interface Motor extends MotorIO {

    /**
     * Commands the motor with a voltage request.
     *
     * @param voltage desired voltage output
     */
    void setVoltage(Voltage voltage);

    /**
     * Commands an open-loop duty cycle.
     *
     * @param speed duty cycle from -1 (full reverse) to 1 (full forward)
     */
    void setSpeed(double speed);

    /**
     * Reports the current mechanism position.
     *
     * @return position in mechanism units
     */
    double getEncoderPosition();

    /**
     * Reports the current mechanism velocity.
     *
     * @return velocity in mechanism units per second
     */
    double getEncoderVelocity();

    /**
     * Reports the applied voltage.
     *
     * @return voltage applied to the motor
     */
    Voltage getVoltage();

    /** Stops the motor output. */
    void stop();

    /**
     * Exposes the underlying motor controller for advanced tuning.
     *
     * @return wrapped SparkMax instance
     */
    SparkMax getMotor();
}
