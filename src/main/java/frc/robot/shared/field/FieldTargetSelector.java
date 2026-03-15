package frc.robot.shared.field;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Selects the active field target for the turret based on the robot's position and alliance color.
 * <p>
 * When the robot is in its alliance zone, the turret aims at the Alliance Hub. When the robot is in the neutral zone (collecting fuel), the turret
 * aims at whichever alliance-zone rally point is closer based on the robot's Y position. All positions are configured from the Blue alliance
 * perspective and mirrored at runtime for Red.
 * </p>
 */
public class FieldTargetSelector {

    private final FieldTargetConfig            config;

    private final Supplier<Pose2d>             robotPoseSupplier;

    private final Supplier<Optional<Alliance>> allianceSupplier;

    /**
     * Creates a field target selector that evaluates position and alliance each time a target is requested.
     *
     * @param config            field target positions and zone boundaries loaded from JSON
     * @param robotPoseSupplier supplier for the robot's current estimated pose in meters and radians
     * @param allianceSupplier  supplier for the current alliance color
     */
    public FieldTargetSelector(
            FieldTargetConfig config,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Optional<Alliance>> allianceSupplier) {
        this.config            = config;
        this.robotPoseSupplier = robotPoseSupplier;
        this.allianceSupplier  = allianceSupplier;
    }

    /**
     * Computes the active field target position based on the robot's current zone and alliance.
     * <p>
     * Call this each cycle to get the turret's tracking target. The returned position accounts for alliance-based coordinate mirroring so callers
     * always receive field-relative meters.
     * </p>
     *
     * @return field-relative target position in meters
     */
    public Translation2d getActiveTargetPosition() {
        Pose2d        robotPose     = robotPoseSupplier.get();
        boolean       isRedAlliance = isRedAlliance();
        double        robotX        = robotPose.getX();

        Translation2d target;
        if (isInAllianceZone(robotX, isRedAlliance)) {
            target = getHubPosition(isRedAlliance);
        } else {
            target = getCloserRallyPoint(robotPose.getY(), isRedAlliance);
        }

        return target;
    }

    /**
     * Returns a human-readable name for the active field target based on the robot's current zone and alliance.
     * <p>
     * Use this for operator telemetry so drivers can see whether the turret is aiming at the hub or a rally point.
     * </p>
     *
     * @return display name of the active target: "Hub", "Left Rally", or "Right Rally"
     */
    public String getActiveTargetName() {
        Pose2d  robotPose     = robotPoseSupplier.get();
        boolean isRedAlliance = isRedAlliance();
        double  robotX        = robotPose.getX();

        if (isInAllianceZone(robotX, isRedAlliance)) {
            return "Hub";
        }

        double leftDistanceY  = Math.abs(robotPose.getY() - config.leftZonePositionYMeters);
        double rightDistanceY = Math.abs(robotPose.getY() - config.rightZonePositionYMeters);
        return leftDistanceY <= rightDistanceY ? "Left Rally" : "Right Rally";
    }

    /**
     * Determines whether the robot is in its own alliance zone.
     *
     * @param robotX        robot's current X position in meters
     * @param isRedAlliance true if the robot is on the Red alliance
     * @return true if the robot is in the alliance zone
     */
    private boolean isInAllianceZone(double robotX, boolean isRedAlliance) {
        if (isRedAlliance) {
            double redBoundary = config.fieldLengthMeters - config.allianceZoneBoundaryXMeters;
            return robotX > redBoundary;
        }
        return robotX < config.allianceZoneBoundaryXMeters;
    }

    /**
     * Returns the Alliance Hub position, mirrored for Red alliance.
     *
     * @param isRedAlliance true if the robot is on the Red alliance
     * @return hub position in field-relative meters
     */
    private Translation2d getHubPosition(boolean isRedAlliance) {
        double x = isRedAlliance ? flipX(config.hubPositionXMeters) : config.hubPositionXMeters;
        return new Translation2d(x, config.hubPositionYMeters);
    }

    /**
     * Returns the rally point closest to the robot's current Y position, mirrored for Red alliance.
     *
     * @param robotY        robot's current Y position in meters
     * @param isRedAlliance true if the robot is on the Red alliance
     * @return closest rally point in field-relative meters
     */
    private Translation2d getCloserRallyPoint(double robotY, boolean isRedAlliance) {
        double leftDistanceY  = Math.abs(robotY - config.leftZonePositionYMeters);
        double rightDistanceY = Math.abs(robotY - config.rightZonePositionYMeters);

        double targetX;
        double targetY;
        if (leftDistanceY <= rightDistanceY) {
            targetX = config.leftZonePositionXMeters;
            targetY = config.leftZonePositionYMeters;
        } else {
            targetX = config.rightZonePositionXMeters;
            targetY = config.rightZonePositionYMeters;
        }

        if (isRedAlliance) {
            targetX = flipX(targetX);
        }

        return new Translation2d(targetX, targetY);
    }

    /**
     * Mirrors an X coordinate for the Red alliance.
     *
     * @param blueX X coordinate from the Blue alliance perspective in meters
     * @return mirrored X coordinate for Red alliance in meters
     */
    private double flipX(double blueX) {
        return config.fieldLengthMeters - blueX;
    }

    /**
     * Checks whether the current alliance is Red, defaulting to Blue if unknown.
     *
     * @return true if Red alliance
     */
    private boolean isRedAlliance() {
        return allianceSupplier.get().orElse(Alliance.Blue) == Alliance.Red;
    }
}
