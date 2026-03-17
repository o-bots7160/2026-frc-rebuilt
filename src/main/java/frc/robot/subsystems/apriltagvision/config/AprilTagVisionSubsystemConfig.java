package frc.robot.subsystems.apriltagvision.config;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.shared.config.AbstractSubsystemConfig;

/**
 * Configuration bundle for the AprilTag vision subsystem. Values are mirrored to SmartDashboard so they can be tuned live without redeploying
 * firmware.
 * <p>
 * Includes the field layout type selector so each robot variant (competition, test, sim) can specify its own AprilTag field layout in the
 * subsystems JSON file.
 * </p>
 */
public class AprilTagVisionSubsystemConfig extends AbstractSubsystemConfig {

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
            this.name          = name;
            this.source        = source;
            this.fieldResource = fieldResource;
        }

        /**
         * Returns the human-readable name of this field layout.
         *
         * @return layout name
         */
        public String getName() {
            return name;
        }

        /**
         * Returns true when this layout comes from the official WPILib field resource.
         *
         * @return true for official layouts, false for custom ones
         */
        public boolean isOfficial() {
            return source == Source.OFFICIAL;
        }

        /**
         * Returns the WPILib resource path for loading the official AprilTag field layout JSON.
         *
         * @return resource path string, or the deploy-relative path for custom layouts
         */
        public String getFieldResource() {
            return fieldResource;
        }
    }

    /**
     * Nested class for JSON deserialization of camera transforms.
     * <p>
     * Translation values are in meters and rotations are in radians.
     * </p>
     */
    public static class CameraTransform {
        /**
         * Robot-to-camera X offset in meters (forward positive).
         */
        public double x;

        /**
         * Robot-to-camera Y offset in meters (left positive).
         */
        public double y;

        /**
         * Robot-to-camera Z offset in meters (up positive).
         */
        public double z;

        /**
         * Robot-to-camera roll in radians.
         */
        public double roll;

        /**
         * Robot-to-camera pitch in radians.
         */
        public double pitch;

        /**
         * Robot-to-camera yaw in radians.
         */
        public double yaw;

        /**
         * Converts this config object to a WPILib Transform3d.
         *
         * @return Transform3d representing the robot-to-camera transform
         */
        public Transform3d toTransform3d() {
            return new Transform3d(
                    new Translation3d(x, y, z),
                    new Rotation3d(roll, pitch, yaw));
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
    public FieldLayoutType              fieldType;

    /**
     * Camera names and their robot-to-camera transforms. Each transform defines the camera position relative to robot center: (x forward, y left, z
     * up).
     */
    public Map<String, CameraTransform> cameras;

    /**
     * Pose quality filters and trust parameters. Grouped into a sub-config so they can be tuned
     * as a unit from the dashboard or adjusted in the deploy JSON.
     */
    public PoseFilterConfig             poseFilter = new PoseFilterConfig();

    /**
     * Returns the configured camera map.
     *
     * @return map of camera names to robot-to-camera transforms
     */
    public Map<String, CameraTransform> getCameras() {
        return cameras;
    }

    /**
     * Returns the pose filter configuration.
     *
     * @return pose filter sub-config with all filtering thresholds
     */
    public PoseFilterConfig getPoseFilter() {
        return poseFilter;
    }

    /**
     * Returns the configured field type selector.
     * <p>
     * Use this to determine which AprilTag field layout is loaded for pose estimation.
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
     * Call this once during subsystem construction so tags can be accessed with {@code getTagPose(int)} for navigation targets.
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
