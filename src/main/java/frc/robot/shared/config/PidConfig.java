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
}
