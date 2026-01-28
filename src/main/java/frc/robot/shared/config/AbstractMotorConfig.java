package frc.robot.shared.config;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;

/**
 * Base configuration bundle for a single motor controller and its mechanism limits.
 * <p>
 * Use subclasses to provide tunable motor settings without coupling device wrappers to a full subsystem config. All values are surfaced through
 * AdvantageKit-backed tunables, so changes can be made live without redeploying.
 * </p>
 */
public abstract class AbstractMotorConfig extends AbstractConfig {

    /** CAN device ID of the motor controller. */
    public int     motorCanId;

    /** True when the motor output should be inverted. */
    public boolean motorInverted;

    /** Smart current limit for the motor in amps. */
    public int     smartCurrentLimitAmps;

    /** Gear ratio expressed as motor rotations per one mechanism rotation. */
    public double  motorRotationsPerMechanismRotation;

    /** True when the motor should enforce forward and reverse soft limits. */
    public boolean useSetpointLimits = true;

    /** Reverse travel soft limit in degrees. */
    public double  reverseSoftLimitDegrees;

    /** Forward travel soft limit in degrees. */
    public double  forwardSoftLimitDegrees;

    /**
     * Supplies the CAN ID (not typically tuned, but exposed for consistency/logging).
     *
     * @return supplier that yields the motor controller CAN ID
     */
    public Supplier<Integer> getMotorCanIdSupplier() {
        return () -> (int) readTunableNumber("motorCanId", motorCanId);
    }

    /**
     * Supplies whether the motor output is inverted.
     *
     * @return supplier that indicates whether to invert motor output
     */
    public Supplier<Boolean> getMotorInvertedSupplier() {
        return () -> readTunableBoolean("motorInverted", motorInverted);
    }

    /**
     * Supplies the smart current limit in amps.
     *
     * @return supplier that yields the current limit in amps
     */
    public Supplier<Integer> getSmartCurrentLimitSupplier() {
        return () -> (int) readTunableNumber("smartCurrentLimitAmps", smartCurrentLimitAmps);
    }

    /**
     * Supplies the gear ratio (motor rotations per mechanism rotation).
     *
     * @return supplier that yields the motor rotations per one mechanism rotation
     */
    public Supplier<Double> getMotorRotationsPerMechanismRotationSupplier() {
        return () -> readTunableNumber("motorRotationsPerMechanismRotation", motorRotationsPerMechanismRotation);
    }

    /**
     * Supplies whether the motor should enforce minimum and maximum setpoint limits.
     *
     * @return supplier that indicates whether setpoint limits are enforced
     */
    public Supplier<Boolean> getUseSetpointLimitsSupplier() {
        return () -> readTunableBoolean("useSetpointLimits", useSetpointLimits);
    }

    /**
     * Supplies the reverse soft limit in degrees.
     *
     * @return supplier that yields the reverse soft limit in degrees
     */
    public Supplier<Double> getReverseSoftLimitDegreesSupplier() {
        return () -> readTunableNumber("reverseSoftLimitDegrees", reverseSoftLimitDegrees);
    }

    /**
     * Supplies the reverse soft limit in radians.
     *
     * @return supplier that yields the reverse soft limit in radians
     */
    public Supplier<Double> getReverseSoftLimitRadiansSupplier() {
        return () -> Units.degreesToRadians(getReverseSoftLimitDegreesSupplier().get());
    }

    /**
     * Supplies the forward soft limit in degrees.
     *
     * @return supplier that yields the forward soft limit in degrees
     */
    public Supplier<Double> getForwardSoftLimitDegreesSupplier() {
        return () -> readTunableNumber("forwardSoftLimitDegrees", forwardSoftLimitDegrees);
    }

    /**
     * Supplies the forward soft limit in radians.
     *
     * @return supplier that yields the forward soft limit in radians
     */
    public Supplier<Double> getForwardSoftLimitRadiansSupplier() {
        return () -> Units.degreesToRadians(getForwardSoftLimitDegreesSupplier().get());
    }
}
