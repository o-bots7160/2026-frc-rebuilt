package frc.robot.shared.config;

import edu.wpi.first.math.util.Units;

/**
 * Configuration values for subsystems that maintain a target velocity using feedforward and PID control.
 * <p>
 * All RPM values represent mechanism (flywheel) speed after gear reduction, not motor shaft speed. The gear ratio is applied at the motor encoder
 * conversion layer in {@link AbstractMotorConfig}, so subsystem code never deals with motor-side rotations. PID and feedforward gains are inherited
 * from {@link AbstractMotorSubsystemConfig}.
 * </p>
 */
public abstract class AbstractVelocitySubsystemConfig extends AbstractMotorSubsystemConfig {

    /**
     * Converts an RPM value to radians per second.
     *
     * @param rpm value in rotations per minute
     * @return equivalent value in radians per second
     */
    public static double rpmToRadiansPerSecond(double rpm) {
        return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    }

    /**
     * Converts an RPM value to degrees per second.
     * <p>
     * Subsystems use this when building simulation motors that expect degrees-per-second suppliers.
     * </p>
     *
     * @param rpm value in rotations per minute
     * @return equivalent value in degrees per second
     */
    public static double rpmToDegreesPerSecond(double rpm) {
        return Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
    }

    /** Maximum allowed mechanism velocity in RPM. Targets above this value are clamped. */
    public double maximumVelocityRpm;

    /** Maximum acceleration in RPM per second for the velocity ramp. Set to 0 to disable the trapezoidal ramp and use direct PID control. */
    public double maximumAccelerationRpmPerSecond;

    /** Acceptable velocity error when deciding if the mechanism is at its target, in RPM. */
    public double velocityToleranceRpm;

    /** How long the velocity must stay within tolerance before reporting ready, in seconds. */
    public double settleTimeSeconds;

    /** Default idle velocity in RPM. Set to 0 to stop the motor when idle. */
    public double idleVelocityRpm;

    /**
     * Returns the maximum mechanism velocity, tuned via SmartDashboard.
     *
     * @return max velocity in RPM
     */
    public double getMaximumVelocityRpm() {
        return readTunableNumber("maximumVelocityRpm", maximumVelocityRpm);
    }

    /**
     * Returns the maximum mechanism velocity in radians per second.
     *
     * @return max velocity in radians per second
     */
    public double getMaximumVelocityRadiansPerSecond() {
        return rpmToRadiansPerSecond(getMaximumVelocityRpm());
    }

    /**
     * Returns the maximum acceleration for the velocity ramp, tuned via SmartDashboard.
     * <p>
     * When this returns 0, the subsystem uses direct PID control without a trapezoidal velocity ramp.
     * </p>
     *
     * @return max acceleration in RPM per second
     */
    public double getMaximumAccelerationRpmPerSecond() {
        return readTunableNumber("maximumAccelerationRpmPerSecond", maximumAccelerationRpmPerSecond);
    }

    /**
     * Returns the maximum acceleration in radians per second squared.
     *
     * @return max acceleration in radians per second squared
     */
    public double getMaximumAccelerationRadiansPerSecondSquared() {
        return rpmToRadiansPerSecond(getMaximumAccelerationRpmPerSecond());
    }

    /**
     * Returns the acceptable velocity error for the at-target check.
     *
     * @return velocity tolerance in RPM
     */
    public double getVelocityToleranceRpm() {
        return readTunableNumber("velocityToleranceRpm", velocityToleranceRpm);
    }

    /**
     * Returns the acceptable velocity error in radians per second.
     *
     * @return velocity tolerance in radians per second
     */
    public double getVelocityToleranceRadiansPerSecond() {
        return rpmToRadiansPerSecond(getVelocityToleranceRpm());
    }

    /**
     * Returns how long the velocity must remain within tolerance before reporting ready.
     *
     * @return settle time in seconds
     */
    public double getSettleTimeSeconds() {
        return readTunableNumber("settleTimeSeconds", settleTimeSeconds);
    }

    /**
     * Returns the default idle velocity, tuned via SmartDashboard.
     *
     * @return idle velocity in RPM
     */
    public double getIdleVelocityRpm() {
        return readTunableNumber("idleVelocityRpm", idleVelocityRpm);
    }

    /**
     * Returns the default idle velocity in radians per second.
     *
     * @return idle velocity in radians per second
     */
    public double getIdleVelocityRadiansPerSecond() {
        return rpmToRadiansPerSecond(getIdleVelocityRpm());
    }
}
