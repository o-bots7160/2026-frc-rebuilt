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
                config.motorConfig,
                ShooterMotor::create,
                ShooterSimMotor::create);

        // When the follower is disabled, operate with the primary motor only.
        if (!config.followerEnabled) {
            return primary;
        }

        Motor follower = buildVelocityMotor(
                config,
                config.followerMotorConfig,
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

    /**
     * Builds an interpolation table mapping distance to time of flight from config data points.
     *
     * @param points array of distance-to-RPM-to-TOF mapping points from config
     * @return populated interpolation table, or an empty table if no TOF data is available
     */
    private static InterpolatingDoubleTreeMap buildDistanceTofTable(DistanceRpmPoint[] points) {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        boolean hasData = false;
        if (points != null) {
            for (DistanceRpmPoint point : points) {
                if (point.timeOfFlightSeconds > 0.0) {
                    table.put(point.distanceMeters, point.timeOfFlightSeconds);
                    hasData = true;
                }
            }
        }
        return hasData ? table : new InterpolatingDoubleTreeMap();
    }

    /**
     * Returns true when at least one distance-RPM point has a positive time-of-flight value.
     *
     * @param points array of distance-to-RPM mapping points from config
     * @return true if TOF data is available
     */
    private static boolean hasTofEntries(DistanceRpmPoint[] points) {
        if (points == null) {
            return false;
        }
        for (DistanceRpmPoint point : points) {
            if (point.timeOfFlightSeconds > 0.0) {
                return true;
            }
        }
        return false;
    }

    private final InterpolatingDoubleTreeMap distanceRpmTable;

    private final InterpolatingDoubleTreeMap distanceTofTable;

    private final boolean                    hasDistanceRpmData;

    private final boolean                    hasTofData;

    /**
     * Operator-controlled RPM offset applied on top of the calculated target each cycle.
     * <p>
     * Positive values boost the flywheel speed; negative values cut it. Reset to zero when the operator releases the trigger.
     * </p>
     */
    private double                           rpmOffsetRpm;

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
        distanceTofTable   = buildDistanceTofTable(config.distanceRpmPoints);
        hasDistanceRpmData = config.distanceRpmPoints != null && config.distanceRpmPoints.length > 0;
        hasTofData         = hasTofEntries(config.distanceRpmPoints);
        rpmOffsetRpm       = 0.0;
    }

    /**
     * Sets the operator RPM offset applied on top of every target velocity.
     * <p>
     * Positive values boost flywheel speed; negative values cut it. Set to zero to clear the adjustment. The offset is added to the raw target
     * inside {@link #setTargetVelocityRpm(double)} before the forward-only clamp, so it takes effect immediately on the next control cycle.
     * </p>
     *
     * @param offsetRpm RPM to add to the calculated target (positive = boost, negative = cut)
     */
    public void setRpmOffset(double offsetRpm) {
        if (isSubsystemDisabled()) {
            logDisabled("setRpmOffset");
            return;
        }
        rpmOffsetRpm = offsetRpm;
        log.recordOutput("rpmOffsetRpm", rpmOffsetRpm);
    }

    /**
     * Returns the current operator RPM offset.
     *
     * @return RPM offset currently applied (positive = boost, negative = cut, zero = none)
     */
    public double getRpmOffset() {
        return rpmOffsetRpm;
    }

    /**
     * Sets a new target velocity, applies the operator RPM offset, and clamps to forward-only rotation.
     * <p>
     * The operator offset is added before clamping so boost and cut adjustments take effect for all shooting modes (distance-based, fixed RPM,
     * continuous). Flywheels should never reverse through the velocity controller, so the result is clamped to zero at the low end.
     * </p>
     *
     * @param targetRpm desired flywheel velocity in RPM (0 to stop, positive to spin forward)
     */
    @Override
    public void setTargetVelocityRpm(double targetRpm) {
        double adjustedRpm    = targetRpm + rpmOffsetRpm;
        double forwardOnlyRpm = MathUtil.clamp(adjustedRpm, 0.0, config.motionProfile.getMaximumVelocityRpm());
        super.setTargetVelocityRpm(forwardOnlyRpm);
    }

    /**
     * Computes the target flywheel RPM for a given distance using the configured interpolation table.
     * <p>
     * The raw interpolated value is scaled by the tunable multiplier and then clamped to the configured maximum velocity. Returns the idle RPM
     * when the interpolation table is empty.
     * </p>
     *
     * @param distanceMeters distance from the robot to the target in meters
     * @return target flywheel RPM, scaled and clamped to the configured maximum
     */
    public double calculateRpmFromDistanceMeters(double distanceMeters) {
        if (isSubsystemDisabled()) {
            logDisabled("calculateRpmFromDistanceMeters");
            return 0.0;
        }

        if (!hasDistanceRpmData || Double.isNaN(distanceMeters) || distanceMeters < 0.0) {
            return config.motionProfile.getIdleVelocityRpm();
        }

        double interpolatedRpm = distanceRpmTable.get(distanceMeters);
        double scaledRpm       = interpolatedRpm * config.getDistanceRpmMultiplier();
        return MathUtil.clamp(scaledRpm, 0.0, config.motionProfile.getMaximumVelocityRpm());
    }

    /**
     * Returns the estimated time of flight for a ball to reach a target at the given distance.
     * <p>
     * The value is interpolated from the configured distance-RPM-TOF lookup table. When no TOF data is configured, returns a conservative linear
     * estimate based on distance. The shoot-on-the-move solver uses this to determine how far the ball drifts during flight.
     * </p>
     *
     * @param distanceMeters distance from the launcher to the target in meters
     * @return estimated time of flight in seconds
     */
    public double getTimeOfFlightSeconds(double distanceMeters) {
        if (hasTofData && !Double.isNaN(distanceMeters) && distanceMeters >= 0.0) {
            return distanceTofTable.get(distanceMeters);
        }
        // Fallback: estimate ~15 m/s average ball speed if no TOF data is configured.
        return distanceMeters / 15.0;
    }

    /**
     * Reports whether the flywheel is spinning at a meaningful shooting velocity and is within tolerance of its current target.
     * <p>
     * Delegates to the base class {@link #isWithinTolerance()} for the velocity error check and additionally verifies that the current target is
     * above idle speed. This prevents false positives when the shooter is still at idle speed.
     * </p>
     *
     * @return true when the flywheel is within tolerance and the target is above idle velocity
     */
    public boolean isAtShootingVelocity() {
        boolean withinTolerance = isWithinTolerance();
        boolean aboveIdle      = getTargetVelocityRpm() > config.motionProfile.getIdleVelocityRpm();
        boolean atShooting     = withinTolerance && aboveIdle;
        log.recordOutput("atShootingVelocity", atShooting);
        return atShooting;
    }
}
