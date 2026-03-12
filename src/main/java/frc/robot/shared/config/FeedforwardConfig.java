package frc.robot.shared.config;

import frc.robot.shared.subsystems.SysIdHelper;

/**
 * Configuration bundle for feedforward gains used by motor control subsystems.
 * <p>
 * Holds the gains that the subsystem consumes directly ({@code kS}, {@code kV}, {@code kA}, {@code kG}). Since {@link SysIdHelper} pre-multiplies
 * position and velocity by 2pi before logging, the SysId analysis tool reports gains in per-radian units that can be entered here directly without
 * correction.
 * </p>
 */
public class FeedforwardConfig extends AbstractConfig {

    /** Static feedforward gain in volts. */
    public double kS;

    /** Velocity feedforward gain in volts per radian per second. */
    public double kV;

    /** Acceleration feedforward gain in volts per radian per second squared. */
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
     * @return kV in volts per radian per second
     */
    public double getkV() {
        return readTunableNumber("kV", kV);
    }

    /**
     * Returns the acceleration feedforward gain, tuned via SmartDashboard.
     *
     * @return kA in volts per radian per second squared
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
}
