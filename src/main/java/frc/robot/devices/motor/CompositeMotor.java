package frc.robot.devices.motor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;

/**
 * Motor adapter that forwards commands to multiple physical motors while presenting a single {@link Motor} interface.
 * <p>
 * Use this when two or more motors are mechanically coupled on the same mechanism (e.g., dual-motor shooter flywheels). One motor is designated the
 * primary; its encoder feedback is used for position, velocity, and telemetry reporting. All other motors are followers that receive identical
 * voltage and speed commands but can have independent configurations (inversion, current limits, gear ratio) defined in their own motor config.
 * </p>
 * <p>
 * This class lives in {@code devices/motor/} so any subsystem that needs paired motors can reuse it. Because the adapter implements {@link Motor},
 * the subsystem hierarchy ({@code AbstractMotorSubsystem}, {@code AbstractVelocitySubsystem}) remains completely unaware of multi-motor wiring.
 * </p>
 *
 * @see Motor
 * @see DisabledMotor
 */
public final class CompositeMotor implements Motor {

    /**
     * Primary motor whose encoder provides position and velocity feedback.
     */
    private final Motor       primary;

    /**
     * Additional motors that receive the same commands as the primary but do not contribute encoder feedback.
     */
    private final List<Motor> followers;

    /**
     * Creates a composite motor from a primary motor and one or more followers.
     * <p>
     * Each motor instance should already be fully configured (CAN ID, inversion, current limits, etc.) before being passed here. The composite does
     * not alter any motor configuration; it only forwards commands.
     * </p>
     *
     * @param primary   motor whose encoder is authoritative for position and velocity readings
     * @param followers additional motors that mirror the primary's voltage and speed commands
     */
    public CompositeMotor(Motor primary, Motor... followers) {
        this.primary = primary;

        List<Motor> followerList = new ArrayList<>();
        for (Motor follower : followers) {
            if (follower != null && !(follower instanceof DisabledMotor)) {
                followerList.add(follower);
            }
        }
        this.followers = Collections.unmodifiableList(followerList);
    }

    /**
     * Sends a voltage command to the primary motor and all followers.
     *
     * @param voltage desired output voltage
     */
    @Override
    public void setVoltage(Voltage voltage) {
        primary.setVoltage(voltage);
        for (Motor follower : followers) {
            follower.setVoltage(voltage);
        }
    }

    /**
     * Sends a duty-cycle command to the primary motor and all followers.
     *
     * @param speed duty cycle from -1 (full reverse) to 1 (full forward)
     */
    @Override
    public void setSpeed(double speed) {
        primary.setSpeed(speed);
        for (Motor follower : followers) {
            follower.setSpeed(speed);
        }
    }

    /**
     * Returns the primary motor's measured position in radians.
     *
     * @return mechanism position from the primary encoder
     */
    @Override
    public double getPositionRadians() {
        return primary.getPositionRadians();
    }

    /**
     * Returns the primary motor's measured velocity in radians per second.
     *
     * @return mechanism velocity from the primary encoder
     */
    @Override
    public double getVelocityRadiansPerSecond() {
        return primary.getVelocityRadiansPerSecond();
    }

    /**
     * Overwrites the encoder position on the primary motor and all followers.
     *
     * @param positionRadians new encoder position in mechanism radians
     */
    @Override
    public void setEncoderPosition(double positionRadians) {
        primary.setEncoderPosition(positionRadians);
        for (Motor follower : followers) {
            follower.setEncoderPosition(positionRadians);
        }
    }

    /**
     * Returns the voltage applied to the primary motor.
     *
     * @return primary motor's applied voltage
     */
    @Override
    public Voltage getVoltage() {
        return primary.getVoltage();
    }

    /**
     * Stops the primary motor and all followers immediately.
     */
    @Override
    public void stop() {
        primary.stop();
        for (Motor follower : followers) {
            follower.stop();
        }
    }

    /**
     * Returns the primary motor's underlying SparkMax for advanced tuning.
     *
     * @return primary motor's SparkMax instance
     */
    @Override
    public SparkMax getMotor() {
        return primary.getMotor();
    }

    /**
     * Updates telemetry inputs from the primary motor's encoder and sensors.
     * <p>
     * Only the primary motor's readings are captured because both motors are mechanically coupled and share the same mechanism state. Follower health
     * (current draw, temperature) can be monitored separately if needed by accessing follower motors directly.
     * </p>
     *
     * @param inputs mutable inputs container to fill for logging
     */
    @Override
    public void updateInputs(MotorIOInputs inputs) {
        primary.updateInputs(inputs);
    }
}
