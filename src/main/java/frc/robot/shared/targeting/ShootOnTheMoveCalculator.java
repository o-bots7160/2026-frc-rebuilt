package frc.robot.shared.targeting;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Shoot-on-the-move solver that computes a compensated aim point and distance for the turret.
 * <p>
 * When the robot is moving, a ball launched from the turret inherits the robot's velocity. During its time of flight the ball drifts in the direction
 * of travel, causing it to overshoot the target if the turret aims straight at it. This solver computes the "virtual" target the turret should aim
 * at so the ball arrives at the real target after accounting for that drift.
 * </p>
 * <p>
 * The math is adapted from the frc-fire-control ShotCalculator (MIT, FRC 5962). The key difference is that this solver returns a compensated
 * {@link Translation2d} aim point for the turret instead of a whole-robot drive heading, because our robot has an independent turret.
 * </p>
 */
public class ShootOnTheMoveCalculator {

    /**
     * Immutable result of the SOTM solve. Contains the compensated aim point, effective distance for RPM lookup, the solved time of flight, and
     * diagnostics.
     *
     * @param compensatedTargetPosition virtual target position the turret should aim at in field coordinates
     * @param compensatedDistanceMeters effective distance from the launcher to the compensated target, used for RPM lookup
     * @param timeOfFlightSeconds       solved time of flight at the compensated distance
     * @param convergenceIterations     number of Newton iterations used (should be 1-3 for typical scenarios)
     * @param sotmActive                true when translational compensation was applied; false when the robot was nearly stationary
     */
    public record ShotSolution(
            Translation2d compensatedTargetPosition,
            double compensatedDistanceMeters,
            double timeOfFlightSeconds,
            int convergenceIterations,
            boolean sotmActive) {

        /** Construct a static (no-SOTM) solution from raw values. */
        public static ShotSolution staticSolution(Translation2d target, double distanceMeters, double tofSeconds) {
            return new ShotSolution(target, distanceMeters, tofSeconds, 0, false);
        }
    }

    private static final double DERIV_H = 0.01;

    private final double dragCoefficient;
    private final double minSpeedMetersPerSecond;
    private final int    maxIterations;
    private final double convergenceToleranceSeconds;

    /** Previous cycle's solved TOF, used to warm-start the Newton solver for faster convergence. */
    private double previousTOF = -1;

    /**
     * Creates a solver with the given tuning parameters.
     *
     * @param dragCoefficient              aerodynamic drag coefficient for the ball's inherited velocity decay during flight (0.47 for a smooth
     *                                     sphere)
     * @param minSpeedMetersPerSecond       robot speed below which SOTM is disabled and the turret aims straight at the target
     * @param maxIterations                maximum Newton iterations before falling back to the initial guess
     * @param convergenceToleranceSeconds  convergence threshold for the Newton solver in seconds
     */
    public ShootOnTheMoveCalculator(
            double dragCoefficient,
            double minSpeedMetersPerSecond,
            int maxIterations,
            double convergenceToleranceSeconds) {
        this.dragCoefficient             = dragCoefficient;
        this.minSpeedMetersPerSecond     = minSpeedMetersPerSecond;
        this.maxIterations               = maxIterations;
        this.convergenceToleranceSeconds = convergenceToleranceSeconds;
    }

    /**
     * Solves for the compensated aim point given the robot's current state.
     * <p>
     * The solver accounts for the turret's physical offset from robot center by computing the launcher velocity as the robot center velocity plus the
     * rotational contribution from the turret offset. This matters because the turret traces an arc when the robot rotates.
     * </p>
     *
     * @param launcherFieldPosition field-relative position of the turret pivot in meters (from
     *                              {@link frc.robot.subsystems.turret.TurretSubsystem#getTurretFieldPosition})
     * @param fieldVelocity         robot center velocity in the field frame (vx, vy, omega)
     * @param turretPivotX          turret pivot X offset from robot center in meters (positive = forward)
     * @param turretPivotY          turret pivot Y offset from robot center in meters (positive = left)
     * @param robotHeadingRadians   current robot heading in radians
     * @param targetPosition        field-relative target position in meters
     * @param tofLookup             function that returns estimated time of flight given a distance in meters
     * @return solved shot solution with compensated aim point and distance
     */
    public ShotSolution solve(
            Translation2d launcherFieldPosition,
            ChassisSpeeds fieldVelocity,
            double turretPivotX,
            double turretPivotY,
            double robotHeadingRadians,
            Translation2d targetPosition,
            DoubleUnaryOperator tofLookup) {

        // Compute launcher field velocity: v_launcher = v_robot + omega x r_offset
        double cosH = Math.cos(robotHeadingRadians);
        double sinH = Math.sin(robotHeadingRadians);
        double fieldOffX = turretPivotX * cosH - turretPivotY * sinH;
        double fieldOffY = turretPivotX * sinH + turretPivotY * cosH;
        double omega = fieldVelocity.omegaRadiansPerSecond;
        double vx = fieldVelocity.vxMetersPerSecond + (-fieldOffY) * omega;
        double vy = fieldVelocity.vyMetersPerSecond + fieldOffX * omega;

        double robotSpeed = Math.hypot(vx, vy);

        // Vector from launcher to target
        double rx = targetPosition.getX() - launcherFieldPosition.getX();
        double ry = targetPosition.getY() - launcherFieldPosition.getY();
        double rawDistance = Math.hypot(rx, ry);

        // Below the minimum speed threshold, skip SOTM and aim directly at the target.
        if (robotSpeed < minSpeedMetersPerSecond) {
            double tof = tofLookup.applyAsDouble(rawDistance);
            previousTOF = tof;
            return ShotSolution.staticSolution(targetPosition, rawDistance, tof);
        }

        // Newton-method solver: find TOF t where f(t) = TOF_LUT(compensated_distance(t)) - t = 0
        double tof;
        if (previousTOF > 0) {
            tof = previousTOF;
        } else {
            tof = tofLookup.applyAsDouble(rawDistance);
        }

        double projDist = rawDistance;
        int iterationsUsed = 0;

        for (int i = 0; i < maxIterations; i++) {
            double prevTOF = tof;

            // Drag-adjusted effective flight time: (1 - e^(-c*t)) / c
            double c = dragCoefficient;
            double dragExp = c < 1e-6 ? 1.0 : Math.exp(-c * tof);
            double driftTOF = c < 1e-6 ? tof : (1.0 - dragExp) / c;

            // Shift the target by the ball's inherited drift
            double prx = rx - vx * driftTOF;
            double pry = ry - vy * driftTOF;
            projDist = Math.hypot(prx, pry);

            // Degenerate guard: ball is on top of the target
            if (projDist < 0.01) {
                tof = tofLookup.applyAsDouble(rawDistance);
                break;
            }

            double lookupTOF = tofLookup.applyAsDouble(projDist);

            // Newton derivative (chain rule)
            double dPrime = -dragExp * (prx * vx + pry * vy) / projDist;
            double gPrime = (tofLookup.applyAsDouble(projDist + DERIV_H) - tofLookup.applyAsDouble(projDist - DERIV_H)) / (2.0 * DERIV_H);
            double f = lookupTOF - tof;
            double fPrime = gPrime * dPrime - 1.0;

            // Newton step with near-zero denominator guard
            if (Math.abs(fPrime) > 0.01) {
                tof = tof - f / fPrime;
            } else {
                tof = lookupTOF;
            }

            // Per-iteration clamp prevents runaway
            tof = MathUtil.clamp(tof, 0.05, 5.0);
            iterationsUsed = i + 1;

            if (Math.abs(tof - prevTOF) < convergenceToleranceSeconds) {
                break;
            }
        }

        // Divergence guard
        if (tof > 5.0 || tof < 0.0 || Double.isNaN(tof)) {
            tof = tofLookup.applyAsDouble(rawDistance);
            projDist = rawDistance;
        }

        previousTOF = tof;

        // Compute the compensated target: shift real target opposite to ball drift
        double finalDriftTOF = dragCoefficient < 1e-6 ? tof : (1.0 - Math.exp(-dragCoefficient * tof)) / dragCoefficient;
        Translation2d compensatedTarget = new Translation2d(
                targetPosition.getX() - vx * finalDriftTOF,
                targetPosition.getY() - vy * finalDriftTOF);

        return new ShotSolution(compensatedTarget, projDist, tof, iterationsUsed, true);
    }

    /**
     * Resets the warm-start state. Call after a pose reset so the solver does not use stale data.
     */
    public void resetWarmStart() {
        previousTOF = -1;
    }
}
