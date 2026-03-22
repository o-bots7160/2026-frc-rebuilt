package frc.robot.subsystems.turret.config;

import frc.robot.shared.config.AbstractSetAndSeekSubsystemConfig;
import frc.robot.shared.field.FieldTargetConfig;

/**
 * Configuration bundle for the turret mechanism. Values are stored in degrees for readability but converted to radians at runtime where needed.
 */
public class TurretSubsystemConfig extends AbstractSetAndSeekSubsystemConfig {

    /** Motor configuration bundle for the turret mechanism. */
    public TurretMotorConfig turretMotorConfig = new TurretMotorConfig();

    /** 3D pivot offset for AdvantageScope component visualization. */
    public ComponentPoseConfig componentPoseConfig = new ComponentPoseConfig();

    /** Field target positions and zone boundaries for zone-aware turret tracking. */
    public FieldTargetConfig fieldTargets = new FieldTargetConfig();

    /**
     * Zero offset between the turret's mechanical zero and robot-forward in degrees.
     * <p>
     * A value of 180 means the turret faces the rear of the robot when at its mechanical zero position.
     * </p>
     */
    public double turretZeroOffsetDegrees = 180.0;

    /**
     * Look-ahead time for rotational velocity compensation in seconds.
     * <p>
     * When the robot is spinning, the turret can lead its aim by this many seconds of predicted heading
     * change to stay on target. Set to zero to disable compensation.
     * </p>
     */
    public double rotationalLeadTimeSeconds = 0.1;

    /**
     * Enables or disables the shoot-on-the-move translational compensation.
     * <p>
     * When enabled, the turret aims at a virtual target that accounts for ball drift during flight due to robot velocity. Disable this at competition
     * if the solver misbehaves.
     * </p>
     */
    public boolean sotmEnabled = true;

    /**
     * Aerodynamic drag coefficient applied to the ball's inherited robot velocity during flight.
     * <p>
     * A typical smooth sphere has a drag coefficient around 0.47. Higher values cause the ball's inherited velocity to decay faster, reducing the
     * SOTM aim correction. Set to 0 to assume no drag (ball keeps full inherited velocity).
     * </p>
     */
    public double sotmDragCoefficient = 0.47;

    /**
     * Robot speed below which SOTM is disabled and the turret aims straight at the target.
     * <p>
     * At very low speeds the ball drift is negligible. Setting this too low causes the solver to apply tiny corrections that add noise without
     * improving accuracy.
     * </p>
     */
    public double sotmMinSpeedMetersPerSecond = 0.1;

    /** Maximum Newton iterations before falling back to the uncompensated TOF. */
    public int sotmMaxIterations = 10;

    /** Convergence threshold for the Newton solver in seconds. */
    public double sotmConvergenceToleranceSeconds = 0.001;

    /**
     * Returns the turret zero offset in degrees.
     * <p>
     * This offset is subtracted from the robot-relative target angle to convert into turret-relative
     * coordinates. A value of 180 means turret mechanical zero points to the rear of the robot.
     * </p>
     *
     * @return turret zero offset in degrees
     */
    public double getTurretZeroOffsetDegrees() {
        return readTunableDegrees("turretZeroOffsetDegrees", turretZeroOffsetDegrees);
    }

    /**
     * Returns the rotational lead time used for yaw-rate compensation.
     * <p>
     * The turret multiplies the robot's yaw rate by this value to predict how far the heading will change
     * and pre-rotates accordingly. Tune this higher if the turret lags behind while the robot spins.
     * </p>
     *
     * @return lead time in seconds
     */
    public double getRotationalLeadTimeSeconds() {
        return readTunableNumber("rotationalLeadTimeSeconds", rotationalLeadTimeSeconds);
    }

    /**
     * Returns whether shoot-on-the-move translational compensation is enabled.
     *
     * @return true when SOTM is active
     */
    public boolean isSotmEnabled() {
        return readTunableBoolean("sotmEnabled", sotmEnabled);
    }

    /**
     * Returns the SOTM drag coefficient for inherited ball velocity decay during flight.
     *
     * @return drag coefficient (dimensionless, 0.47 for a smooth sphere)
     */
    public double getSotmDragCoefficient() {
        return readTunableNumber("sotmDragCoefficient", sotmDragCoefficient);
    }

    /**
     * Returns the minimum robot speed required to activate SOTM compensation.
     *
     * @return speed threshold in meters per second
     */
    public double getSotmMinSpeedMetersPerSecond() {
        return readTunableNumber("sotmMinSpeedMetersPerSecond", sotmMinSpeedMetersPerSecond);
    }

    /**
     * Returns the maximum Newton iterations for the SOTM solver.
     *
     * @return maximum iteration count
     */
    public int getSotmMaxIterations() {
        return sotmMaxIterations;
    }

    /**
     * Returns the convergence tolerance for the SOTM Newton solver.
     *
     * @return tolerance in seconds
     */
    public double getSotmConvergenceToleranceSeconds() {
        return sotmConvergenceToleranceSeconds;
    }
}
