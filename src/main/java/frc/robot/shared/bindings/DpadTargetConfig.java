package frc.robot.shared.bindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.shared.config.AbstractConfig;

/**
 * Configuration for a single d-pad pathfinding target. Stores a field position and heading in blue-alliance coordinates that the robot will
 * pathfind to when the corresponding d-pad button is held.
 * <p>
 * Set {@code enabled} to {@code true} and populate the pose fields in the deploy JSON to activate a d-pad direction. When disabled, the
 * corresponding button binding is skipped entirely.
 * </p>
 */
public class DpadTargetConfig extends AbstractConfig {

    /**
     * X-coordinate of the target position on the field in meters (blue-alliance perspective).
     * <p>
     * Positive X points toward the red alliance wall. This value is flipped at runtime when the robot is on the red alliance.
     * </p>
     */
    public double  xMeters        = 0.0;

    /**
     * Y-coordinate of the target position on the field in meters (blue-alliance perspective).
     * <p>
     * Positive Y points toward the left wall when facing the red alliance wall.
     * </p>
     */
    public double  yMeters        = 0.0;

    /**
     * Target heading in degrees (blue-alliance perspective).
     * <p>
     * 0 degrees faces the red alliance wall, 180 degrees faces the blue alliance wall. Flipped at runtime for red alliance.
     * </p>
     */
    public double  headingDegrees = 0.0;

    /**
     * Whether this d-pad target is active.
     * <p>
     * When {@code false}, the corresponding d-pad button will not be bound to any command. Set to {@code true} in the deploy JSON after configuring
     * the target pose.
     * </p>
     */
    public boolean enabled        = false;

    /**
     * Reads the tunable X-coordinate of the target in meters.
     *
     * @return X position on the field in meters (blue-alliance perspective)
     */
    public double getXMeters() {
        return readTunableNumber("xMeters", xMeters);
    }

    /**
     * Reads the tunable Y-coordinate of the target in meters.
     *
     * @return Y position on the field in meters (blue-alliance perspective)
     */
    public double getYMeters() {
        return readTunableNumber("yMeters", yMeters);
    }

    /**
     * Reads the tunable heading in degrees.
     *
     * @return target heading in degrees (blue-alliance perspective)
     */
    public double getHeadingDegrees() {
        return readTunableDegrees("headingDegrees", headingDegrees);
    }

    /**
     * Reads whether this d-pad target is enabled.
     *
     * @return {@code true} if the target is active and should be bound to the d-pad button
     */
    public boolean getEnabled() {
        return readTunableBoolean("enabled", enabled);
    }

    /**
     * Converts the configured target to a WPILib Pose2d in blue-alliance coordinates.
     * <p>
     * Callers are responsible for flipping the returned pose for the red alliance using {@code FlippingUtil.flipFieldPose()}.
     * </p>
     *
     * @return field pose with position in meters and heading in degrees converted to a Rotation2d
     */
    public Pose2d toPose2d() {
        return new Pose2d(getXMeters(), getYMeters(), Rotation2d.fromDegrees(getHeadingDegrees()));
    }
}
