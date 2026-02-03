package frc.robot.shared.config;

import java.util.Objects;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Configuration for selecting the official AprilTag field layout and origin.
 */
public class FieldLayoutConfig extends AbstractConfig {

    /**
     * Enumerates the supported AprilTag field layouts for the 2026 game.
     */
    public enum FieldLayoutType {
        /** The welded field layout. */
        WELDED,
        /** The AndyMark field layout. */
        ANDYMARK
    }

    /** Enum name of the welded field layout to load. */
    public static final String WELDED_FIELD = "k2026RebuiltWelded";

    /** Enum name of the AndyMark field layout to load. */
    public static final String ANDY_MARK_FIELD = "k2026RebuiltAndymark";

    /** Enum name for the AprilTag layout origin to apply. */
    public static final String ORIGIN = "kBlueAllianceWallRightSide";

    /** Field type selector used to pick between welded and AndyMark layouts. */
    public FieldLayoutType fieldType;

    /**
     * Returns the configured field type selector.
     * <p>
     * Use this to determine which official field layout is loaded for AprilTag poses.
     * </p>
     *
     * @return selected field layout type
     */
    public FieldLayoutType getFieldType() {
        return parseFieldType(readTunableString("fieldType", requireFieldType().name()));
    }

    /**
     * Builds an AprilTag field layout using the configured field type and origin.
     * <p>
     * Use this after loading the JSON config so tags can be accessed with {@code getTagPose(int)} for navigation targets.
     * </p>
     *
     * @return loaded AprilTag field layout with the configured origin applied
     * @throws IllegalArgumentException when the configured enum names are invalid
     */
    public AprilTagFieldLayout loadLayout() {
        FieldLayoutType     resolvedFieldType = getFieldType();
        AprilTagFields      selectedField     = resolveSelectedField(resolvedFieldType);
        AprilTagFieldLayout layout            = AprilTagFieldLayout.loadField(selectedField);
        layout.setOrigin(resolveOrigin());
        return layout;
    }

    private AprilTagFields resolveSelectedField(FieldLayoutType fieldTypeValue) {
        FieldLayoutType resolvedType = Objects.requireNonNull(fieldTypeValue, "fieldType must be configured");
        if (resolvedType == FieldLayoutType.ANDYMARK) {
            return AprilTagFields.valueOf(ANDY_MARK_FIELD);
        }
        return AprilTagFields.valueOf(WELDED_FIELD);
    }

    private AprilTagFieldLayout.OriginPosition resolveOrigin() {
        return AprilTagFieldLayout.OriginPosition.valueOf(ORIGIN);
    }

    private FieldLayoutType requireFieldType() {
        return Objects.requireNonNull(fieldType, "fieldType must be configured");
    }

    private FieldLayoutType parseFieldType(String value) {
        String normalized = Objects.requireNonNull(value, "fieldType must be configured").trim().toLowerCase();
        if ("andymark".equals(normalized) || "andy-mark".equals(normalized) || "andy".equals(normalized)) {
            return FieldLayoutType.ANDYMARK;
        }
        if ("welded".equals(normalized)) {
            return FieldLayoutType.WELDED;
        }
        return FieldLayoutType.valueOf(value.trim());
    }
}
