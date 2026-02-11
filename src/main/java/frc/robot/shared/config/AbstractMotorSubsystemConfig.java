package frc.robot.shared.config;

/**
 * Configuration values shared by any motor-driven subsystem that uses PID control and feedforward estimation.
 * <p>
 * Concrete configs for position-based (set-and-seek) and velocity-based subsystems extend this class to inherit tunable PID and feedforward gains
 * without duplicating the boilerplate. All gains are surfaced to SmartDashboard so they can be tuned live without redeploying.
 * </p>
 */
public abstract class AbstractMotorSubsystemConfig extends AbstractConfig {

    /** Proportional gain for the controller. */
    public double kP;

    /** Integral gain for the controller. */
    public double kI;

    /** Derivative gain for the controller. */
    public double kD;

    /** Static feedforward gain (volts). */
    public double kS;

    /** Velocity feedforward gain (volts per mechanism unit per second). */
    public double kV;

    /** Acceleration feedforward gain (volts per mechanism unit per second squared). */
    public double kA;

    /**
     * Returns the proportional gain for the controller.
     *
     * @return kP
     */
    public double getkP() {
        return readTunableNumber("kP", kP);
    }

    /**
     * Returns the integral gain for the controller.
     *
     * @return kI
     */
    public double getkI() {
        return readTunableNumber("kI", kI);
    }

    /**
     * Returns the derivative gain for the controller.
     *
     * @return kD
     */
    public double getkD() {
        return readTunableNumber("kD", kD);
    }

    /**
     * Returns the static feedforward term.
     *
     * @return kS (volts)
     */
    public double getkS() {
        return readTunableNumber("kS", kS);
    }

    /**
     * Returns the velocity feedforward term.
     *
     * @return kV (volts per mechanism unit per second)
     */
    public double getkV() {
        return readTunableNumber("kV", kV);
    }

    /**
     * Returns the acceleration feedforward term.
     *
     * @return kA (volts per mechanism unit per second squared)
     */
    public double getkA() {
        return readTunableNumber("kA", kA);
    }
}
