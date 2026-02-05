package frc.robot.shared.config;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Locale;
import java.util.Objects;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Configuration for selecting the official AprilTag field layout and origin.
 */
public class FieldLayoutConfig extends AbstractConfig {

    /**
     * Enumerates the supported AprilTag field layouts for the 2026 game.
     */
    public enum FieldLayoutType {
        /** The welded field layout. */
        WELDED("welded", Source.OFFICIAL, WELDED_FIELD),
        /** The AndyMark field layout. */
        ANDYMARK("andymark", Source.OFFICIAL, ANDY_MARK_FIELD),
        /** Obots shop. */
        SHOP("shop", Source.CUSTOM, SHOP_FIELD);

        private enum Source {
            OFFICIAL,
            CUSTOM
        }

        private final String name;
        private final Source source;
        private final String fieldResource;

        FieldLayoutType(String name, Source source, String fieldResource) {
            this.name   = name;
            this.source = source;
            this.fieldResource  = fieldResource;
        }

        public String getName() {
            return name;
        }

        public boolean isOfficial() {
            return source == Source.OFFICIAL;
        }

        public String getFieldResource() {
            return fieldResource;
        }
    }

    /** Enum name of the welded field layout to load. */
    public static final String WELDED_FIELD = "k2026RebuiltWelded";

    /** Enum name of the AndyMark field layout to load. */
    public static final String ANDY_MARK_FIELD = "k2026RebuiltAndymark";

    /** Deploy-relative path for the shop field layout. */
    public static final String SHOP_FIELD = "shop-field-layout.json";

    /** Enum name for the AprilTag layout origin to apply. */
    public static final String ORIGIN = "kBlueAllianceWallRightSide";

    /** Field type selector used to pick between field layouts. */
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
        AprilTagFieldLayout layout            = loadLayoutForType(resolvedFieldType);
        layout.setOrigin(resolveOrigin());
        return layout;
    }

    private AprilTagFieldLayout loadLayoutForType(FieldLayoutType fieldTypeValue) {
        FieldLayoutType resolvedType = Objects.requireNonNull(fieldTypeValue, "fieldType must be configured");
        if (resolvedType.isOfficial()) {
            AprilTagFields selectedField = AprilTagFields.valueOf(resolvedType.getFieldResource());
            return AprilTagFieldLayout.loadField(selectedField);
        }
        return loadCustomLayout(resolvedType.getFieldResource());
    }

    private AprilTagFieldLayout loadCustomLayout(String resourcePath) {
        if (resourcePath == null || resourcePath.isBlank()) {
            throw new IllegalArgumentException("Custom field layouts must define a resource path");
        }
        Path deployPath = Filesystem.getDeployDirectory().toPath().resolve(resourcePath.trim());
        try {
            return new AprilTagFieldLayout(deployPath);
        } catch (IOException e) {
            throw new IllegalStateException("Failed to load custom field layout from deploy path: " + deployPath, e);
        }
    }

    private AprilTagFieldLayout.OriginPosition resolveOrigin() {
        return AprilTagFieldLayout.OriginPosition.valueOf(ORIGIN);
    }

    private FieldLayoutType requireFieldType() {
        return Objects.requireNonNull(fieldType, "fieldType must be configured");
    }

    private FieldLayoutType parseFieldType(String value) {
        String normalized = Objects.requireNonNull(value, "fieldType must be configured").trim().toLowerCase(Locale.ROOT);
        if ("andymark".equals(normalized) || "andy-mark".equals(normalized) || "andy".equals(normalized)) {
            return FieldLayoutType.ANDYMARK;
        }
        if ("shop".equals(normalized)) {
            return FieldLayoutType.SHOP;
        }
        if ("welded".equals(normalized)) {
            return FieldLayoutType.WELDED;
        }
        for (FieldLayoutType type : FieldLayoutType.values()) {
            if (type.getName().equals(normalized)) {
                return type;
            }
        }
        return FieldLayoutType.valueOf(value.trim());
    }
}
