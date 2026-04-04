package frc.robot.shared.config;

/**
 * Configuration bundle for PID controller gains.
 * <p>
 * Embed an instance of this class in any motor subsystem config that uses a PID controller. All three gains are surfaced to SmartDashboard so they
 * can be tuned live without redeploying.
 * </p>
 */
public class PidConfig extends AbstractConfig {

    /** Proportional gain for the controller. */
    public double kP;

    /** Integral gain for the controller. */
    public double kI;

    /** Derivative gain for the controller. */
    public double kD;

    /**
     * Maximum error (in process-variable units) at which the integral term accumulates.
     * <p>
     * When the absolute error exceeds this value, the integral accumulator is reset to zero. This prevents the integral term from winding up during
     * large transients (such as spin-up) where integral action would cause overshoot. Set to zero to disable IZone (the integral accumulates at all
     * error magnitudes). The value is in the same units as the PID measurement — radians per second for velocity loops, radians for position loops.
     * </p>
     */
    public double iZone;

    /**
     * Maximum absolute value of the integral term's output contribution in volts.
     * <p>
     * The WPILib PIDController clamps the integral accumulator so that {@code kI * accumulator} never exceeds this range. This prevents integral
     * windup from producing large voltage spikes after sustained error. Set to zero to use the WPILib default range of ±1.0.
     * </p>
     */
    public double integratorRangeMax;

    /**
     * Returns the proportional gain, tuned via SmartDashboard.
     *
     * @return kP
     */
    public double getkP() {
        return readTunableNumber("kP", kP);
    }

    /**
     * Returns the integral gain, tuned via SmartDashboard.
     *
     * @return kI
     */
    public double getkI() {
        return readTunableNumber("kI", kI);
    }

    /**
     * Returns the derivative gain, tuned via SmartDashboard.
     *
     * @return kD
     */
    public double getkD() {
        return readTunableNumber("kD", kD);
    }

    /**
     * Returns the IZone threshold, tuned via SmartDashboard.
     * <p>
     * A value of zero means IZone is disabled and the integral accumulates at all error magnitudes.
     * </p>
     *
     * @return IZone threshold in process-variable units (rad/s for velocity, rad for position), or zero to disable
     */
    public double getIZone() {
        return readTunableNumber("iZone", iZone);
    }

    /**
     * Returns the maximum integrator output range, tuned via SmartDashboard.
     * <p>
     * A value of zero means the WPILib default range of ±1.0 is used.
     * </p>
     *
     * @return maximum absolute integrator output in volts, or zero for the default
     */
    public double getIntegratorRangeMax() {
        return readTunableNumber("integratorRangeMax", integratorRangeMax);
    }
}
