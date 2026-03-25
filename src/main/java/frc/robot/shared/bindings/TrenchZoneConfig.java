package frc.robot.shared.bindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.shared.config.AbstractConfig;

/**
 * Defines a rectangular trench zone on the field and provides geometric detection for whether a straight-line path crosses it. When a d-pad
 * pathfinding route intersects a trench zone, intermediate waypoints are inserted at the zone entry and exit so the robot passes through with the
 * correct heading.
 * <p>
 * All coordinates are in field (blue-alliance) frame. Each hub has two trenches (north and south), giving four zones total. The X boundaries cover
 * the hub width with a buffer, and the Y boundaries split at the hub center so the correct trench center Y is used for waypoints. Each zone's
 * {@code trenchCenterYMeters} is derived from the AprilTags at the trench opening (e.g., tags 22/23 for the blue north trench at y=7.411). The
 * heading rule is purely directional: 180 degrees for +X travel, 0 degrees for -X travel.
 * </p>
 */
public class TrenchZoneConfig extends AbstractConfig {

    // Cohen-Sutherland region code bit flags.
    private static final int INSIDE = 0;

    private static final int LEFT   = 1;

    private static final int RIGHT  = 2;

    private static final int BOTTOM = 4;

    private static final int TOP    = 8;

    /**
     * Computes the Cohen-Sutherland region code for a point relative to the AABB.
     *
     * @param x    point X
     * @param y    point Y
     * @param xMin box minimum X
     * @param yMin box minimum Y
     * @param xMax box maximum X
     * @param yMax box maximum Y
     * @return region code indicating which edges the point is outside of
     */
    private static int computeRegionCode(double x, double y, double xMin, double yMin, double xMax, double yMax) {
        int code = INSIDE;
        if (x < xMin) {
            code |= LEFT;
        } else if (x > xMax) {
            code |= RIGHT;
        }
        if (y < yMin) {
            code |= BOTTOM;
        } else if (y > yMax) {
            code |= TOP;
        }
        return code;
    }

    /**
     * Whether this trench zone is active.
     * <p>
     * Disabled zones are skipped during intersection checks.
     * </p>
     */
    public boolean enabled = false;

    /**
     * Minimum X-coordinate of the trench zone in meters (field coordinates).
     */
    public double minXMeters = 0.0;

    /**
     * Maximum X-coordinate of the trench zone in meters (field coordinates).
     */
    public double maxXMeters = 0.0;

    /**
     * Minimum Y-coordinate of the trench zone in meters (field coordinates).
     */
    public double minYMeters = 0.0;

    /**
     * Maximum Y-coordinate of the trench zone in meters (field coordinates).
     */
    public double maxYMeters = 0.0;

    /**
     * Y-coordinate of the trench center in meters (field coordinates), derived from the AprilTags centered on the trench opening.
     * <p>
     * Both entry and exit waypoints use this Y value so the robot lines up with the center of the trench passage.
     * </p>
     */
    public double trenchCenterYMeters = 4.199;

    /**
     * Buffer distance in meters added outside the zone boundary when computing entry and exit waypoints.
     * <p>
     * The entry waypoint is placed this far outside the zone edge so the robot finishes its heading correction before physically entering the
     * trench. A value of 0.5 meters gives roughly half a robot length of margin.
     * </p>
     */
    public double bufferMeters = 0.5;

    /**
     * Reads whether this trench zone is enabled.
     *
     * @return {@code true} if the zone should be checked during intersection tests
     */
    public boolean getEnabled() {
        return readTunableBoolean("enabled", enabled);
    }

    /**
     * Reads the tunable minimum X-coordinate of the zone in meters.
     *
     * @return minimum X boundary in meters
     */
    public double getMinXMeters() {
        return readTunableNumber("minXMeters", minXMeters);
    }

    /**
     * Reads the tunable maximum X-coordinate of the zone in meters.
     *
     * @return maximum X boundary in meters
     */
    public double getMaxXMeters() {
        return readTunableNumber("maxXMeters", maxXMeters);
    }

    /**
     * Reads the tunable minimum Y-coordinate of the zone in meters.
     *
     * @return minimum Y boundary in meters
     */
    public double getMinYMeters() {
        return readTunableNumber("minYMeters", minYMeters);
    }

    /**
     * Reads the tunable maximum Y-coordinate of the zone in meters.
     *
     * @return maximum Y boundary in meters
     */
    public double getMaxYMeters() {
        return readTunableNumber("maxYMeters", maxYMeters);
    }
    /**
     * Reads the tunable trench center Y-coordinate in meters.
     *
     * @return trench center Y in meters
     */
    public double getTrenchCenterYMeters() {
        return readTunableNumber("trenchCenterYMeters", trenchCenterYMeters);
    }

    /**
     * Reads the tunable buffer distance in meters.
     *
     * @return buffer distance outside the zone edge in meters
     */
    public double getBufferMeters() {
        return readTunableNumber("bufferMeters", bufferMeters);
    }
    /**
     * Tests whether the straight-line segment from one pose to another intersects this trench zone rectangle.
     * <p>
     * Uses the Cohen-Sutherland line-clipping algorithm to test a line segment against the axis-aligned bounding box. This is a conservative
     * heuristic: false positives (the line crosses the zone but the PathPlanner AD* pathfinder routes around) cause a harmless detour through the
     * entry/exit waypoints, while false negatives are dangerous. Size the zones generously to minimize false negatives.
     * </p>
     *
     * @param start  starting pose in field coordinates (typically the robot's current odometry pose)
     * @param target target pose in field coordinates (already alliance-flipped)
     * @return {@code true} if the line segment from start to target crosses this zone
     */
    public boolean intersectsLineSegment(Pose2d start, Pose2d target) {
        if (!getEnabled()) {
            return false;
        }

        return intersectsLineSegment(
                start.getX(), start.getY(),
                target.getX(), target.getY());
    }
    /**
     * Computes the entry waypoint just outside the trench zone on the side closest to the starting position.
     * <p>
     * The waypoint is placed at the zone's X-edge (plus buffer) on the start side, at the configured trench center Y derived from the AprilTags
     * centered on the trench opening. The heading is set to the required trench heading based on travel direction: 180 degrees for +X travel,
     * 0 degrees for -X travel.
     * </p>
     *
     * @param start  starting pose in field coordinates
     * @param target target pose in field coordinates
     * @return entry waypoint pose with the required heading for trench traversal
     */
    public Pose2d computeEntryWaypoint(Pose2d start, Pose2d target) {
        double     buffer         = getBufferMeters();
        double     centerY        = getTrenchCenterYMeters();
        boolean    travelingPlusX = target.getX() > start.getX();
        double     entryX         = travelingPlusX ? getMinXMeters() - buffer : getMaxXMeters() + buffer;
        Rotation2d heading        = Rotation2d.fromDegrees(travelingPlusX ? 180.0 : 0.0);

        return new Pose2d(entryX, centerY, heading);
    }
    /**
     * Computes the exit waypoint just outside the trench zone on the side closest to the target position.
     * <p>
     * The waypoint is placed at the zone's X-edge (plus buffer) on the target side, at the configured trench center Y so the robot exits aligned
     * with the trench passage. The heading matches the entry heading so the robot maintains a consistent orientation through the entire trench
     * transit.
     * </p>
     *
     * @param start  starting pose in field coordinates
     * @param target target pose in field coordinates
     * @return exit waypoint pose with the required heading for trench traversal
     */
    public Pose2d computeExitWaypoint(Pose2d start, Pose2d target) {
        double     buffer         = getBufferMeters();
        double     centerY        = getTrenchCenterYMeters();
        boolean    travelingPlusX = target.getX() > start.getX();
        double     exitX          = travelingPlusX ? getMaxXMeters() + buffer : getMinXMeters() - buffer;
        Rotation2d heading        = Rotation2d.fromDegrees(travelingPlusX ? 180.0 : 0.0);

        return new Pose2d(exitX, centerY, heading);
    }

    /**
     * Tests whether a line segment from (x1, y1) to (x2, y2) intersects the axis-aligned bounding box defined by the zone boundaries.
     * <p>
     * Implements the Cohen-Sutherland line-clipping algorithm. The line segment is clipped against each edge of the AABB in turn. If any portion of
     * the segment remains after clipping against all four edges, the segment intersects the box.
     * </p>
     *
     * @param x1 start X of the line segment
     * @param y1 start Y of the line segment
     * @param x2 end X of the line segment
     * @param y2 end Y of the line segment
     * @return {@code true} if the segment intersects the zone rectangle
     */
    private boolean intersectsLineSegment(double x1, double y1, double x2, double y2) {
        double xMin = getMinXMeters();
        double xMax = getMaxXMeters();
        double yMin = getMinYMeters();
        double yMax = getMaxYMeters();

        int code1 = computeRegionCode(x1, y1, xMin, yMin, xMax, yMax);
        int code2 = computeRegionCode(x2, y2, xMin, yMin, xMax, yMax);

        while (true) {
            if ((code1 | code2) == 0) {
                // Both endpoints inside the box.
                return true;
            }

            if ((code1 & code2) != 0) {
                // Both endpoints share an outside zone — no intersection.
                return false;
            }

            // Pick the endpoint outside the box.
            int outsideCode = (code1 != 0) ? code1 : code2;
            double x;
            double y;

            if ((outsideCode & TOP) != 0) {
                x = x1 + (x2 - x1) * (yMax - y1) / (y2 - y1);
                y = yMax;
            } else if ((outsideCode & BOTTOM) != 0) {
                x = x1 + (x2 - x1) * (yMin - y1) / (y2 - y1);
                y = yMin;
            } else if ((outsideCode & RIGHT) != 0) {
                y = y1 + (y2 - y1) * (xMax - x1) / (x2 - x1);
                x = xMax;
            } else {
                y = y1 + (y2 - y1) * (xMin - x1) / (x2 - x1);
                x = xMin;
            }

            if (outsideCode == code1) {
                x1 = x;
                y1 = y;
                code1 = computeRegionCode(x1, y1, xMin, yMin, xMax, yMax);
            } else {
                x2 = x;
                y2 = y;
                code2 = computeRegionCode(x2, y2, xMin, yMin, xMax, yMax);
            }
        }
    }
}
