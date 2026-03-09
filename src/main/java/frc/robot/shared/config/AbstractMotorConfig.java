package frc.robot.shared.config;

import edu.wpi.first.math.util.Units;

/**
 * Base configuration bundle for a single motor controller and its mechanism limits.
 * <p>
 * Values are deserialized from JSON and exposed through plain getters. Motor configuration is applied once during initialization; changes require a
 * redeploy.
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
     * Returns the CAN ID of the motor controller.
     *
     * @return motor controller CAN ID
     */
    public int getMotorCanId() {
        return motorCanId;
    }

    /**
     * Returns whether the motor output is inverted.
     *
     * @return true when the motor output is inverted
     */
    public boolean getMotorInverted() {
        return motorInverted;
    }

    /**
     * Returns the smart current limit in amps.
     *
     * @return current limit in amps
     */
    public int getSmartCurrentLimitAmps() {
        return smartCurrentLimitAmps;
    }

    /**
     * Returns the gear ratio (motor rotations per mechanism rotation).
     *
     * @return motor rotations per one mechanism rotation
     */
    public double getMotorRotationsPerMechanismRotation() {
        return motorRotationsPerMechanismRotation;
    }

    /**
     * Returns whether the motor should enforce minimum and maximum setpoint limits.
     *
     * @return true when setpoint limits are enforced
     */
    public boolean getUseSetpointLimits() {
        return useSetpointLimits;
    }

    /**
     * Returns the reverse soft limit in degrees.
     *
     * @return reverse soft limit in degrees
     */
    public double getReverseSoftLimitDegrees() {
        return reverseSoftLimitDegrees;
    }

    /**
     * Returns the reverse soft limit in radians.
     *
     * @return reverse soft limit in radians
     */
    public double getReverseSoftLimitRadians() {
        return Units.degreesToRadians(reverseSoftLimitDegrees);
    }

    /**
     * Returns the forward soft limit in degrees.
     *
     * @return forward soft limit in degrees
     */
    public double getForwardSoftLimitDegrees() {
        return forwardSoftLimitDegrees;
    }

    /**
     * Returns the forward soft limit in radians.
     *
     * @return forward soft limit in radians
     */
    public double getForwardSoftLimitRadians() {
        return Units.degreesToRadians(forwardSoftLimitDegrees);
    }
}
