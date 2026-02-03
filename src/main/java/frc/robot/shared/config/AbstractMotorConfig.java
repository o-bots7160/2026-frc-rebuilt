package frc.robot.shared.config;

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
     * Returns the CAN ID (not typically tuned, but exposed for consistency/logging).
     *
     * @return motor controller CAN ID
     */
    public int getMotorCanId() {
        return (int) readTunableNumber("motorCanId", motorCanId);
    }

    /**
     * Returns whether the motor output is inverted.
     *
     * @return true when the motor output is inverted
     */
    public boolean getMotorInverted() {
        return readTunableBoolean("motorInverted", motorInverted);
    }

    /**
     * Returns the smart current limit in amps.
     *
     * @return current limit in amps
     */
    public int getSmartCurrentLimitAmps() {
        return (int) readTunableNumber("smartCurrentLimitAmps", smartCurrentLimitAmps);
    }

    /**
     * Returns the gear ratio (motor rotations per mechanism rotation).
     *
     * @return motor rotations per one mechanism rotation
     */
    public double getMotorRotationsPerMechanismRotation() {
        return readTunableNumber("motorRotationsPerMechanismRotation", motorRotationsPerMechanismRotation);
    }

    /**
     * Returns whether the motor should enforce minimum and maximum setpoint limits.
     *
     * @return true when setpoint limits are enforced
     */
    public boolean getUseSetpointLimits() {
        return readTunableBoolean("useSetpointLimits", useSetpointLimits);
    }

    /**
     * Returns the reverse soft limit in degrees.
     *
     * @return reverse soft limit in degrees
     */
    public double getReverseSoftLimitDegrees() {
        return readTunableDegrees("reverseSoftLimitDegrees", reverseSoftLimitDegrees);
    }

    /**
     * Returns the reverse soft limit in radians.
     *
     * @return reverse soft limit in radians
     */
    public double getReverseSoftLimitRadians() {
        return readTunableDegreesAsRadians("reverseSoftLimitDegrees", reverseSoftLimitDegrees);
    }

    /**
     * Returns the forward soft limit in degrees.
     *
     * @return forward soft limit in degrees
     */
    public double getForwardSoftLimitDegrees() {
        return readTunableDegrees("forwardSoftLimitDegrees", forwardSoftLimitDegrees);
    }

    /**
     * Returns the forward soft limit in radians.
     *
     * @return forward soft limit in radians
     */
    public double getForwardSoftLimitRadians() {
        return readTunableDegreesAsRadians("forwardSoftLimitDegrees", forwardSoftLimitDegrees);
    }
}
