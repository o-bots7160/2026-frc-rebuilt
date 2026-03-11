package frc.robot.shared.config;

/**
 * Configuration bundle for feedforward gains used by motor control subsystems.
 * <p>
 * Holds the corrected gains that the subsystem consumes directly ({@code kS}, {@code kV}, {@code kA}, {@code kG}) and the raw SysId tool outputs
 * ({@code sysIdKs}, {@code sysIdKv}, {@code sysIdKa}) for auditing. On startup, {@link #validateSysIdGains(String)} checks that the corrected
 * velocity and acceleration gains match the raw SysId outputs divided by 2pi, warning operators if they appear uncorrected.
 * </p>
 * <p>
 * The 2pi correction is necessary because WPILib's SysId logging layer internally converts radians/sec to rotations/sec before writing to the log
 * file, but the SysId analysis tool reads those values as if they were still in radians/sec. This inflates kV and kA by a factor of 2pi. Use
 * {@link #convertSysIdGain(double)} to compute the corrected value from a raw SysId output.
 * </p>
 */
public class FeedforwardConfig extends AbstractConfig {

    /**
     * Factor used to convert raw SysId gains from per-rotation units to per-radian units.
     */
    private static final double TWO_PI = 2.0 * Math.PI;

    /**
     * Tolerance fraction (5 percent) used when validating corrected gains against raw SysId outputs.
     */
    private static final double VALIDATION_TOLERANCE = 0.05;

    /**
     * Converts a raw SysId gain from per-rotation units to per-radian units by dividing by 2pi.
     * <p>
     * Use this after reading gains from the SysId analysis tool to compute the corrected kV or kA value for the subsystem config.
     * </p>
     *
     * @param rawGain the raw gain value reported by the SysId tool
     * @return the corrected gain value in per-radian units
     */
    public static double convertSysIdGain(double rawGain) {
        return rawGain / TWO_PI;
    }

    /**
     * Checks whether two values are within the configured tolerance of each other.
     */
    private static boolean isWithinTolerance(double actual, double expected) {
        if (expected == 0.0) {
            return actual == 0.0;
        }
        return Math.abs(actual - expected) / Math.abs(expected) <= VALIDATION_TOLERANCE;
    }

    /** Static feedforward gain in volts. Not affected by the 2pi conversion. */
    public double kS;

    /** Velocity feedforward gain in volts per radian per second (corrected). */
    public double kV;

    /** Acceleration feedforward gain in volts per radian per second squared (corrected). */
    public double kA;

    /**
     * Gravity feedforward gain in volts.
     * <p>
     * This is the voltage needed to hold an arm-type mechanism stationary against gravity when horizontal. Set to 0 for non-arm mechanisms
     * (flywheels, belts, turrets). WPILib's {@link edu.wpi.first.math.controller.ArmFeedforward} multiplies kG by the cosine of the angle from
     * horizontal.
     * </p>
     */
    public double kG;

    /**
     * Raw static feedforward gain as reported by the SysId tool (volts).
     * <p>
     * kS is not affected by the 2pi conversion, so this should match {@link #kS} exactly.
     * </p>
     */
    public double sysIdKs;

    /**
     * Raw velocity feedforward gain as reported by the SysId tool (volts per rotation per second).
     * <p>
     * Divide by 2pi to get the corrected {@link #kV} in volts per radian per second.
     * </p>
     */
    public double sysIdKv;

    /**
     * Raw acceleration feedforward gain as reported by the SysId tool (volts per rotation per second squared).
     * <p>
     * Divide by 2pi to get the corrected {@link #kA} in volts per radian per second squared.
     * </p>
     */
    public double sysIdKa;

    /**
     * Returns the static feedforward gain, tuned via SmartDashboard.
     *
     * @return kS in volts
     */
    public double getkS() {
        return readTunableNumber("kS", kS);
    }

    /**
     * Returns the velocity feedforward gain, tuned via SmartDashboard.
     *
     * @return kV in volts per radian per second (corrected)
     */
    public double getkV() {
        return readTunableNumber("kV", kV);
    }

    /**
     * Returns the acceleration feedforward gain, tuned via SmartDashboard.
     *
     * @return kA in volts per radian per second squared (corrected)
     */
    public double getkA() {
        return readTunableNumber("kA", kA);
    }

    /**
     * Returns the gravity feedforward gain, tuned via SmartDashboard.
     *
     * @return kG in volts (voltage to hold the mechanism horizontal against gravity)
     */
    public double getkG() {
        return readTunableNumber("kG", kG);
    }

    /**
     * Validates that the corrected feedforward gains match the raw SysId outputs divided by 2pi.
     * <p>
     * Call this at startup after configuration is loaded. If raw SysId values are present (non-zero) and the corrected values do not match within a
     * 5 percent tolerance, a warning is reported to the Driver Station so operators know the gains may need correction.
     * </p>
     *
     * @param subsystemName the name of the subsystem for the warning message (e.g., "FeederSubsystem")
     */
    public void validateSysIdGains(String subsystemName) {
        if (sysIdKv > 0.0) {
            double expectedKv = convertSysIdGain(sysIdKv);
            if (!isWithinTolerance(kV, expectedKv)) {
                RobotEnvironment.reportWarning(
                        subsystemName + ": feedforward kV (" + kV + ") does not match sysIdKv / 2pi ("
                                + expectedKv + "). Raw SysId kV = " + sysIdKv
                                + ". Did you forget to divide by 2pi?",
                        false);
            }
        }

        if (sysIdKa > 0.0) {
            double expectedKa = convertSysIdGain(sysIdKa);
            if (!isWithinTolerance(kA, expectedKa)) {
                RobotEnvironment.reportWarning(
                        subsystemName + ": feedforward kA (" + kA + ") does not match sysIdKa / 2pi ("
                                + expectedKa + "). Raw SysId kA = " + sysIdKa
                                + ". Did you forget to divide by 2pi?",
                        false);
            }
        }

        if (sysIdKs > 0.0 && !isWithinTolerance(kS, sysIdKs)) {
            RobotEnvironment.reportWarning(
                    subsystemName + ": feedforward kS (" + kS + ") does not match sysIdKs ("
                            + sysIdKs + "). kS should not be divided by 2pi.",
                    false);
        }
    }
}
