package frc.robot.shared.field;

/**
 * Field target positions used for zone-aware turret tracking.
 * <p>
 * All positions are defined from the Blue alliance perspective in meters. Red alliance coordinates are computed at runtime by mirroring the X-axis.
 * Embed this config in the turret subsystem config so positions are loaded from the deploy JSON. Divider coordinates are derived from AprilTag
 * positions on the field structure: AprilTag 26 marks the field center and AprilTag 23 marks the neutral zone boundary.
 * </p>
 */
public class FieldTargetConfig {

    /** Total field length along the X-axis in meters. */
    public double fieldLengthMeters           = 16.54;

    /** Total field width along the Y-axis in meters. */
    public double fieldWidthMeters            = 8.21;

    /**
     * X-coordinate boundary that separates the alliance zone from the neutral zone in meters.
     * <p>
     * When the robot's X position is less than this value (Blue alliance), the robot is considered in the alliance zone. For Red alliance, the
     * boundary is mirrored to fieldLength minus this value. Derived from AprilTag 23's X position (4.574 m).
     * </p>
     */
    public double allianceZoneBoundaryXMeters = 4.574;

    /**
     * Y-coordinate of the field center divider in meters.
     * <p>
     * This is the lateral midline of the field, derived from AprilTag 26's Y position (4.021 m). Use it for determining whether the robot is on the
     * left or right side of the field.
     * </p>
     */
    public double fieldCenterDividerYMeters   = 4.021;

    /**
     * X-coordinate of the Alliance Hub target in meters (Blue alliance perspective).
     * <p>
     * Derived from the midpoint between AprilTag 26 (x=4.008 m) and AprilTag 20 (x=5.215 m), which sit on opposite sides of the hub.
     * </p>
     */
    public double hubPositionXMeters          = 4.612;

    /** Y-coordinate of the Alliance Hub target in meters, aligned with the field center divider. */
    public double hubPositionYMeters          = 4.021;

    /** X-coordinate of the left zone rally point in meters (Blue alliance perspective). */
    public double leftZonePositionXMeters     = 2.0;

    /** Y-coordinate of the left zone rally point in meters. */
    public double leftZonePositionYMeters     = 6.5;

    /** X-coordinate of the right zone rally point in meters (Blue alliance perspective). */
    public double rightZonePositionXMeters    = 2.0;

    /** Y-coordinate of the right zone rally point in meters. */
    public double rightZonePositionYMeters    = 1.7;
}
