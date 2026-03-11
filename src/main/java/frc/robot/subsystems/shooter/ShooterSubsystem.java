package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.devices.motor.CompositeMotor;
import frc.robot.devices.motor.Motor;
import frc.robot.shared.subsystems.AbstractVelocitySubsystem;
import frc.robot.subsystems.shooter.config.DistanceRpmPoint;
import frc.robot.subsystems.shooter.config.ShooterSubsystemConfig;
import frc.robot.subsystems.shooter.devices.ShooterMotor;
import frc.robot.subsystems.shooter.devices.ShooterSimMotor;

/**
 * Shooter subsystem that spins a flywheel to launch Fuel into the scoring hub.
 * <p>
 * The competition robot uses two mechanically coupled motors: a primary and a follower. The follower can be independently inverted and
 * current-limited via its own config block. On robots with only one shooter motor (e.g., the test robot), set
 * {@code followerEnabled = false} and the subsystem operates with a single motor transparently.
 * </p>
 * <p>
 * All RPM values in the public API represent flywheel (mechanism) speed after gear reduction, not motor shaft speed. The gear ratio is applied at the
 * motor encoder conversion layer, so callers never deal with motor-side rotations.
 * </p>
 * <p>
 * The subsystem uses a feedforward model to estimate the voltage needed for a target velocity and a PID controller to correct for disturbances (such
 * as when a piece enters the shooter and momentarily slows the flywheel). An optional trapezoidal velocity ramp provides smooth spin-up when
 * configured.
 * </p>
 */
public class ShooterSubsystem extends AbstractVelocitySubsystem<ShooterSubsystemConfig> {

    /**
     * Builds the correct motor configuration for the shooter, wrapping two motors in a {@link CompositeMotor} when the follower is enabled, or
     * returning a single motor when it is not.
     *
     * @param config shooter subsystem config containing both motor config bundles
     * @return configured motor (composite or single), or null when the subsystem is disabled
     */
    private static Motor buildShooterMotor(ShooterSubsystemConfig config) {
        if (!config.enabled) {
            return null;
        }

        Motor primary = buildVelocityMotor(
                config,
                config.shooterMotorConfig,
                ShooterMotor::create,
                ShooterSimMotor::create);

        // When the follower is disabled, operate with the primary motor only.
        if (!config.followerEnabled) {
            return primary;
        }

        Motor follower = buildVelocityMotor(
                config,
                config.shooterFollowerMotorConfig,
                ShooterMotor::create,
                ShooterSimMotor::create);

        return new CompositeMotor(primary, follower);
    }

    /**
     * Builds the interpolation table from config distance-RPM data points.
     *
     * @param points array of distance-to-RPM mapping points from config
     * @return populated interpolation table, or an empty table if no points are configured
     */
    private static InterpolatingDoubleTreeMap buildDistanceRpmTable(DistanceRpmPoint[] points) {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        if (points != null) {
            for (DistanceRpmPoint point : points) {
                table.put(point.distanceMeters, point.rpm);
            }
        }
        return table;
    }

    private final InterpolatingDoubleTreeMap distanceRpmTable;

    private final boolean                    hasDistanceRpmData;

    /**
     * Builds the shooter subsystem with a primary motor and an optional follower motor.
     * <p>
     * When the follower motor config is enabled, both motors are wrapped in a {@link CompositeMotor} so the subsystem hierarchy sees a single
     * {@link Motor}. When the follower is disabled, only the primary motor is used.
     * </p>
     *
     * @param config shooter configuration bundle loaded from JSON; velocities are expressed in RPM
     */
    public ShooterSubsystem(ShooterSubsystemConfig config) {
        super(config, buildShooterMotor(config));
        distanceRpmTable   = buildDistanceRpmTable(config.distanceRpmPoints);
        hasDistanceRpmData = config.distanceRpmPoints != null && config.distanceRpmPoints.length > 0;
    }

    /**
     * Sets a new target velocity, clamped to forward-only rotation.
     * <p>
     * Flywheels should never reverse through the velocity controller. Negative values are clamped to zero before delegating to the base class.
     * </p>
     *
     * @param targetRpm desired flywheel velocity in RPM (0 to stop, positive to spin forward)
     */
    @Override
    public void setTargetVelocityRpm(double targetRpm) {
        double forwardOnlyRpm = MathUtil.clamp(targetRpm, 0.0, config.getMaximumVelocityRpm());
        super.setTargetVelocityRpm(forwardOnlyRpm);
    }

    /**
     * Computes the target flywheel RPM for a given distance using the configured interpolation table.
     * <p>
     * The raw interpolated value is scaled by the tunable multiplier and then clamped to the configured minimum and maximum shooting RPM. Returns the
     * idle RPM when the interpolation table is empty.
     * </p>
     *
     * @param distanceMeters distance from the robot to the target in meters
     * @return target flywheel RPM, scaled and clamped to configured limits
     */
    public double calculateRpmFromDistanceMeters(double distanceMeters) {
        if (isSubsystemDisabled()) {
            logDisabled("calculateRpmFromDistanceMeters");
            return 0.0;
        }

        if (!hasDistanceRpmData) {
            return config.getIdleVelocityRpm();
        }

        double interpolatedRpm = distanceRpmTable.get(distanceMeters);
        double scaledRpm       = interpolatedRpm * config.getDistanceRpmMultiplier();
        return MathUtil.clamp(scaledRpm, config.getMinimumShootingRpm(), config.getMaximumShootingRpm());
    }
}
