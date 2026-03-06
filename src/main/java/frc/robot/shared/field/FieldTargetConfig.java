package frc.robot.shared.field;

/**
 * Field target positions used for zone-aware turret tracking.
 * <p>
 * All positions are defined from the Blue alliance perspective in meters. Red alliance coordinates are computed at runtime by mirroring the X-axis.
 * Embed this config in the turret subsystem config so positions are loaded from the deploy JSON.
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
     * boundary is mirrored to fieldLength minus this value.
     * </p>
     */
    public double allianceZoneBoundaryXMeters = 5.5;

    /** X-coordinate of the Alliance Hub target in meters (Blue alliance perspective). */
    public double hubPositionXMeters          = 3.0;

    /** Y-coordinate of the Alliance Hub target in meters. */
    public double hubPositionYMeters          = 4.1;

    /** X-coordinate of the left zone rally point in meters (Blue alliance perspective). */
    public double leftZonePositionXMeters     = 2.0;

    /** Y-coordinate of the left zone rally point in meters. */
    public double leftZonePositionYMeters     = 6.5;

    /** X-coordinate of the right zone rally point in meters (Blue alliance perspective). */
    public double rightZonePositionXMeters    = 2.0;

    /** Y-coordinate of the right zone rally point in meters. */
    public double rightZonePositionYMeters    = 1.7;
}
