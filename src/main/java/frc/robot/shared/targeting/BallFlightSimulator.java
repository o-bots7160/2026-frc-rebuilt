package frc.robot.shared.targeting;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.shared.logging.Logger;
import frc.robot.subsystems.shooter.config.DistanceRpmPoint;

/**
 * Simulation-only utility that models Fuel launched from the shooter as 3D projectiles.
 * <p>
 * Each cycle the simulator checks whether the shooter and indexer are ready to fire. When they are, a new ball is spawned at the turret exit with a
 * velocity derived from the current flywheel RPM. All active balls advance through simple projectile kinematics (gravity + drag) and their positions
 * are logged as a {@code Pose3d[]} for AdvantageScope's 3D field view using the Fuel game piece type.
 * </p>
 * <p>
 * This class is not a subsystem and does not participate in the command scheduler. It is instantiated in {@code RobotContainer} only when
 * {@code RobotEnvironment.isSimulation()} is true, and its {@link #periodic()} method is called from the container's periodic loop.
 * </p>
 */
public class BallFlightSimulator {

    /**
     * A single in-flight ball with initial conditions and elapsed time.
     *
     * @param x0  initial field X position in meters
     * @param y0  initial field Y position in meters
     * @param z0  initial height in meters
     * @param vx0 initial field X velocity in meters per second
     * @param vy0 initial field Y velocity in meters per second
     * @param vz0 initial vertical velocity in meters per second (positive = up)
     * @param age elapsed time since launch in seconds
     */
    private record SimBall(double x0, double y0, double z0, double vx0, double vy0, double vz0, double age) {

        /**
         * Advances the ball by one time step.
         *
         * @param dt time step in seconds
         * @return new ball with updated age
         */
        SimBall tick(double dt) {
            return new SimBall(x0, y0, z0, vx0, vy0, vz0, age + dt);
        }

        /**
         * Computes the current 3D position using kinematic equations with gravity and drag.
         *
         * @param gravity gravitational acceleration in meters per second squared (positive value, applied downward)
         * @param drag    aerodynamic drag coefficient applied to horizontal velocity decay
         * @return current position as a Pose3d for AdvantageScope rendering
         */
        Pose3d position(double gravity, double drag) {
            double t = age;

            // Horizontal velocity decays exponentially due to drag: v(t) = v0 * e^(-c*t)
            // Position integral: x(t) = v0 * (1 - e^(-c*t)) / c
            double driftTime;
            if (drag < 1e-6) {
                driftTime = t;
            } else {
                driftTime = (1.0 - Math.exp(-drag * t)) / drag;
            }

            double x = x0 + vx0 * driftTime;
            double y = y0 + vy0 * driftTime;
            double z = z0 + vz0 * t - 0.5 * gravity * t * t;

            return new Pose3d(x, y, Math.max(z, 0.0), new Rotation3d());
        }

        /**
         * Returns true when the ball should be removed from the simulation.
         *
         * @param gravity    gravitational acceleration in meters per second squared
         * @param drag       drag coefficient
         * @param maxAgeSecs maximum age before forced removal
         * @return true if the ball hit the ground or exceeded max age
         */
        boolean isExpired(double gravity, double drag, double maxAgeSecs) {
            if (age > maxAgeSecs) {
                return true;
            }
            double z = z0 + vz0 * age - 0.5 * gravity * age * age;
            return age > 0.1 && z <= 0.0;
        }
    }

    private static final double DT_SECONDS                = 0.02;

    private static final double GRAVITY_MPS2              = 9.81;

    private static final double MAX_BALL_AGE_SECONDS      = 5.0;

    private static final int    MAX_ACTIVE_BALLS          = 20;

    private static final double LAUNCH_ANGLE_DEGREES      = 50.0;

    private static final double LAUNCH_ANGLE_RADIANS      = Units.degreesToRadians(LAUNCH_ANGLE_DEGREES);

    private static final double MIN_SHOT_COOLDOWN_SECONDS = 0.20;

    /**
     * Builds an interpolation table mapping flywheel RPM to estimated ball exit velocity in meters per second.
     * <p>
     * The exit velocity is derived from the configured distance-RPM-TOF data points. For each point, the average horizontal speed is
     * {@code distance / TOF}, and the total exit speed is {@code horizontal / cos(launchAngle)}. The table maps the RPM at each distance to this
     * derived exit speed so we can look up velocity from the current measured RPM at runtime.
     * </p>
     *
     * @param points distance-RPM-TOF calibration data from the shooter config
     * @return interpolation table from RPM to exit velocity in meters per second
     */
    private static InterpolatingDoubleTreeMap buildRpmToVelocityTable(DistanceRpmPoint[] points) {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

        // Anchor at 0 RPM so velocity scales linearly down from the lowest
        // calibration point instead of clamping to it.
        table.put(0.0, 0.0);

        if (points != null) {
            for (DistanceRpmPoint point : points) {
                if (point.timeOfFlightSeconds > 0.0 && point.distanceMeters > 0.0) {
                    double horizontalSpeed = point.distanceMeters / point.timeOfFlightSeconds;
                    double totalSpeed      = horizontalSpeed / Math.cos(LAUNCH_ANGLE_RADIANS);
                    table.put(point.rpm, totalSpeed);
                }
            }
        }

        // Fallback: if no valid points were added, provide a reasonable default
        boolean hasEntries = false;
        if (points != null) {
            for (DistanceRpmPoint point : points) {
                if (point.timeOfFlightSeconds > 0.0 && point.distanceMeters > 0.0) {
                    hasEntries = true;
                    break;
                }
            }
        }
        if (!hasEntries) {
            table.put(1000.0, 5.0);
            table.put(4000.0, 12.0);
        }
        return table;
    }

    private final Logger                     log               = Logger.getInstance(BallFlightSimulator.class.getSimpleName());

    private final Supplier<Pose2d>           robotPoseSupplier;

    private final Supplier<Double>           turretAngleDegreesSupplier;

    private final Supplier<Double>           shooterRpmSupplier;

    private final Supplier<Boolean>          shooterReadySupplier;

    private final Supplier<Boolean>          indexerFeedingSupplier;

    private final Supplier<ChassisSpeeds>    fieldVelocitySupplier;

    private final double                     turretZeroOffsetDegrees;

    private final double                     turretPivotX;

    private final double                     turretPivotY;

    private final double                     turretPivotZ;

    private final double                     dragCoefficient;

    /** Maps shooter RPM to estimated ball exit velocity in meters per second. */
    private final InterpolatingDoubleTreeMap rpmToExitVelocityTable;

    private final List<SimBall>              activeBalls       = new ArrayList<>();

    private int                              totalLaunched     = 0;

    private double                           shotCooldownTimer = 0.0;

    /**
     * Creates the ball flight simulator with suppliers for all required robot state.
     *
     * @param robotPoseSupplier          supplier for the current robot pose in meters and radians
     * @param turretAngleDegreesSupplier supplier for the turret's measured angle in degrees relative to its mechanical zero
     * @param shooterRpmSupplier         supplier for the current measured flywheel RPM
     * @param shooterReadySupplier       supplier returning true when the shooter is at shooting velocity
     * @param indexerFeedingSupplier     supplier returning true when the indexer is actively feeding forward
     * @param fieldVelocitySupplier      supplier for the robot's field-relative velocity
     * @param turretZeroOffsetDegrees    turret zero offset in degrees (180 means turret faces rear at mechanical zero)
     * @param turretPivotX               turret pivot X offset from robot center in meters (positive = forward)
     * @param turretPivotY               turret pivot Y offset from robot center in meters (positive = left)
     * @param turretPivotZ               turret pivot height in meters
     * @param dragCoefficient            aerodynamic drag coefficient for the ball in flight
     * @param distanceRpmPoints          distance-RPM-TOF calibration data used to derive exit velocity from RPM
     */
    public BallFlightSimulator(
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Double> turretAngleDegreesSupplier,
            Supplier<Double> shooterRpmSupplier,
            Supplier<Boolean> shooterReadySupplier,
            Supplier<Boolean> indexerFeedingSupplier,
            Supplier<ChassisSpeeds> fieldVelocitySupplier,
            double turretZeroOffsetDegrees,
            double turretPivotX,
            double turretPivotY,
            double turretPivotZ,
            double dragCoefficient,
            DistanceRpmPoint[] distanceRpmPoints) {
        this.robotPoseSupplier          = robotPoseSupplier;
        this.turretAngleDegreesSupplier = turretAngleDegreesSupplier;
        this.shooterRpmSupplier         = shooterRpmSupplier;
        this.shooterReadySupplier       = shooterReadySupplier;
        this.indexerFeedingSupplier     = indexerFeedingSupplier;
        this.fieldVelocitySupplier      = fieldVelocitySupplier;
        this.turretZeroOffsetDegrees    = turretZeroOffsetDegrees;
        this.turretPivotX               = turretPivotX;
        this.turretPivotY               = turretPivotY;
        this.turretPivotZ               = turretPivotZ;
        this.dragCoefficient            = dragCoefficient;
        this.rpmToExitVelocityTable     = buildRpmToVelocityTable(distanceRpmPoints);
    }

    /**
     * Advances all in-flight balls, checks for new launches, and logs positions to AdvantageKit.
     * <p>
     * Call once per robot loop from {@code RobotContainer.periodic()}.
     * </p>
     */
    public void periodic() {
        // Advance cooldown timer
        shotCooldownTimer = Math.max(0.0, shotCooldownTimer - DT_SECONDS);

        // Check for new ball launch
        if (shouldLaunch()) {
            launchBall();
        }

        // Advance and prune active balls
        List<SimBall> updated = new ArrayList<>(activeBalls.size());
        for (SimBall ball : activeBalls) {
            SimBall advanced = ball.tick(DT_SECONDS);
            if (!advanced.isExpired(GRAVITY_MPS2, dragCoefficient, MAX_BALL_AGE_SECONDS)) {
                updated.add(advanced);
            }
        }
        activeBalls.clear();
        activeBalls.addAll(updated);

        // Log ball positions for AdvantageScope
        Pose3d[] poses = new Pose3d[activeBalls.size()];
        for (int i = 0; i < activeBalls.size(); i++) {
            poses[i] = activeBalls.get(i).position(GRAVITY_MPS2, dragCoefficient);
        }
        log.recordOutput("balls", poses);
        log.recordOutput("activeBalls", activeBalls.size());
        log.recordOutput("totalLaunched", totalLaunched);
    }

    /**
     * Returns true when the shooter and indexer are both ready and the cooldown has elapsed.
     */
    private boolean shouldLaunch() {
        boolean shooterReady   = shooterReadySupplier.get();
        boolean indexerFeeding = indexerFeedingSupplier.get();
        boolean cooldownReady  = shotCooldownTimer <= 0.0;
        boolean poolAvailable  = activeBalls.size() < MAX_ACTIVE_BALLS;

        log.recordOutput("launch/shooterReady", shooterReady);
        log.recordOutput("launch/indexerFeeding", indexerFeeding);
        log.recordOutput("launch/cooldownReady", cooldownReady);
        log.recordOutput("launch/shooterRpm", shooterRpmSupplier.get());

        return cooldownReady && poolAvailable && shooterReady && indexerFeeding;
    }

    /**
     * Spawns a new ball at the turret exit with a velocity derived from the current flywheel RPM and turret aim direction.
     */
    private void launchBall() {
        Pose2d        robotPose        = robotPoseSupplier.get();
        double        turretDegrees    = turretAngleDegreesSupplier.get();
        double        shooterRpm       = shooterRpmSupplier.get();
        ChassisSpeeds fieldVelocity    = fieldVelocitySupplier.get();

        // Compute turret field-relative aim angle
        double        robotHeadingRad  = robotPose.getRotation().getRadians();
        double        turretRad        = Units.degreesToRadians(turretDegrees);
        double        zeroOffsetRad    = Units.degreesToRadians(turretZeroOffsetDegrees);
        double        turretFieldAngle = robotHeadingRad + turretRad + zeroOffsetRad;

        // Compute turret pivot position on the field
        double        cosH             = Math.cos(robotHeadingRad);
        double        sinH             = Math.sin(robotHeadingRad);
        double        fieldX           = robotPose.getX() + turretPivotX * cosH - turretPivotY * sinH;
        double        fieldY           = robotPose.getY() + turretPivotX * sinH + turretPivotY * cosH;

        // Derive ball exit velocity from RPM
        double        exitSpeed        = rpmToExitVelocityTable.get(Math.abs(shooterRpm));
        double        vHorizontal      = exitSpeed * Math.cos(LAUNCH_ANGLE_RADIANS);
        double        vVertical        = exitSpeed * Math.sin(LAUNCH_ANGLE_RADIANS);

        // Split horizontal velocity into field X/Y using turret aim direction
        double        vx               = vHorizontal * Math.cos(turretFieldAngle);
        double        vy               = vHorizontal * Math.sin(turretFieldAngle);

        // Add robot velocity for momentum inheritance
        vx += fieldVelocity.vxMetersPerSecond;
        vy += fieldVelocity.vyMetersPerSecond;

        SimBall ball = new SimBall(fieldX, fieldY, turretPivotZ, vx, vy, vVertical, 0.0);
        activeBalls.add(ball);
        totalLaunched++;
        shotCooldownTimer = MIN_SHOT_COOLDOWN_SECONDS;

        log.recordOutput("lastLaunchVelocityMps", exitSpeed);
        log.recordOutput("lastTurretFieldAngleDegrees", Units.radiansToDegrees(turretFieldAngle));
    }
}
